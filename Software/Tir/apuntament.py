"""
tir/apuntament.py — Control de servos PAN/TILT i sistema de disparo
====================================================================
Responsabilitats:
  - Controladors de servo: DryRun / GPIO / PCA9685
  - Controlador PID per a pan i tilt
  - Classe FireController (disparar quan s'és a punt)
  - Classe AimController (unifica PID + servos + disparo)

Paràmetres físics:
  - PAN  : gira tota la part superior ±30° respecte al centre (60°–120°)
  - TILT : inclina el canó ±10° respecte al centre (80°–100°)

Ús:
    from tir.apuntament import AimController
    aim = AimController(cfg, dry_run=False)
    aim.actualitzar(error_x, error_y, on_target, area_px, frame_w, frame_h)
    aim.centre()
    aim.cleanup()
"""

import sys
import time
import math
import logging
from typing import Optional

log = logging.getLogger("robot.tir")


def _install(pkgs):
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", *pkgs, "-q"])


# ===========================================================================
# UTILITATS PWM
# ===========================================================================

def _deg_to_duty(deg: float, cfg) -> float:
    """Converteix graus a duty cycle (%) per a RPi.GPIO PWM."""
    period_us = 1_000_000.0 / cfg.pwm_freq
    pulse_us = cfg.servo_min_us + (deg / 180.0) * (cfg.servo_max_us - cfg.servo_min_us)
    return (pulse_us / period_us) * 100.0


def _deg_to_pca_ticks(deg: float, cfg) -> int:
    """Converteix graus a ticks PCA9685 (0–4095)."""
    pulse_us = cfg.servo_min_us + (deg / 180.0) * (cfg.servo_max_us - cfg.servo_min_us)
    return max(0, min(4095, int((pulse_us / 20000.0) * 4096)))


# ===========================================================================
# CONTROLADOR PID
# ===========================================================================

class PID:
    def __init__(self, kp: float, ki: float, kd: float, max_output: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = time.time()

    def update(self, error: float) -> float:
        now = time.time()
        dt = max(now - self._last_time, 1e-3)
        self._integral = max(-50.0, min(50.0, self._integral + error * dt))
        derivative = (error - self._last_error) / dt
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        self._last_error = error
        self._last_time = now
        return max(-self.max_output, min(self.max_output, output))

    def reset(self):
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = time.time()


# ===========================================================================
# CONTROLADORS DE SERVO (base + implementacions)
# ===========================================================================

class _ServoBase:
    def set_pan(self, deg: float): raise NotImplementedError
    def set_tilt(self, deg: float): raise NotImplementedError
    def cleanup(self): pass


class _DryRun(_ServoBase):
    def __init__(self):
        log.info("Servos: MODE DRY-RUN (no es mouran físicament)")

    def set_pan(self, deg: float):
        log.debug(f"[DRY] PAN  → {deg:.1f}°")

    def set_tilt(self, deg: float):
        log.debug(f"[DRY] TILT → {deg:.1f}°")


class _GPIO(_ServoBase):
    """GPIO directe amb PWM per software. Pins recomanats: 12, 13 (hardware PWM)."""

    def __init__(self, cfg):
        try:
            import RPi.GPIO as GPIO
        except ImportError:
            _install(["RPi.GPIO"]); import RPi.GPIO as GPIO
        self._GPIO = GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(cfg.gpio_pan_pin,  GPIO.OUT)
        GPIO.setup(cfg.gpio_tilt_pin, GPIO.OUT)
        self._pan  = GPIO.PWM(cfg.gpio_pan_pin,  cfg.pwm_freq)
        self._tilt = GPIO.PWM(cfg.gpio_tilt_pin, cfg.pwm_freq)
        self._pan.start(_deg_to_duty(cfg.pan_center_deg, cfg))
        self._tilt.start(_deg_to_duty(cfg.tilt_center_deg, cfg))
        self._cfg = cfg
        log.info(f"GPIO servos: pan=GPIO{cfg.gpio_pan_pin}, tilt=GPIO{cfg.gpio_tilt_pin}")

    def set_pan(self, deg: float):
        self._pan.ChangeDutyCycle(_deg_to_duty(deg, self._cfg))

    def set_tilt(self, deg: float):
        self._tilt.ChangeDutyCycle(_deg_to_duty(deg, self._cfg))

    def cleanup(self):
        self._pan.stop()
        self._tilt.stop()
        self._GPIO.cleanup()


class _PCA9685(_ServoBase):
    """PCA9685 via I2C. Recomanat per a producció (no carrega la CPU)."""

    def __init__(self, cfg):
        try:
            from adafruit_pca9685 import PCA9685
            import board, busio
        except ImportError:
            _install(["adafruit-circuitpython-pca9685", "adafruit-blinka"])
            from adafruit_pca9685 import PCA9685
            import board, busio
        import busio as _busio, board as _board
        i2c = _busio.I2C(_board.SCL, _board.SDA)
        self._pca = PCA9685(i2c, address=cfg.pca_i2c_address)
        self._pca.frequency = cfg.pwm_freq
        self._cfg = cfg
        self._pan_ch  = cfg.pca_pan_channel
        self._tilt_ch = cfg.pca_tilt_channel
        self.set_pan(cfg.pan_center_deg)
        self.set_tilt(cfg.tilt_center_deg)
        log.info(f"PCA9685: addr=0x{cfg.pca_i2c_address:02X} "
                 f"pan=ch{self._pan_ch} tilt=ch{self._tilt_ch}")

    def _dc(self, ticks: int) -> int:
        return int(ticks * 65535 / 4095)

    def set_pan(self, deg: float):
        self._pca.channels[self._pan_ch].duty_cycle = self._dc(
            _deg_to_pca_ticks(deg, self._cfg))

    def set_tilt(self, deg: float):
        self._pca.channels[self._tilt_ch].duty_cycle = self._dc(
            _deg_to_pca_ticks(deg, self._cfg))

    def cleanup(self):
        self._pca.deinit()


def _crear_servo(modo: str, dry_run: bool, cfg) -> _ServoBase:
    if dry_run:
        return _DryRun()
    if modo == "gpio":
        return _GPIO(cfg)
    if modo == "pca9685":
        return _PCA9685(cfg)
    # Auto-detect
    for cls in (_PCA9685, _GPIO):
        try:
            ctrl = cls(cfg)
            log.info(f"Servo auto-detectat: {cls.__name__}")
            return ctrl
        except Exception:
            pass
    log.warning("No s'ha pogut connectar cap controlador de servo. Activant dry-run.")
    return _DryRun()


# ===========================================================================
# CONTROLADOR DE DISPARO
# ===========================================================================

class FireController:
    """
    Gestiona el motor de disparo (un sol pols GPIO).

    El disparo s'activa quan el robot porta `fire_on_target_frames` frames
    consecutius amb l'objectiu centrat (on_target=True).
    """

    def __init__(self, cfg, dry_run: bool = False):
        self._cfg = cfg
        self._dry_run = dry_run
        self._on_target_count = 0
        self._last_fire_time = 0.0
        self._GPIO = None

        if cfg.fire_enabled and not dry_run:
            try:
                import RPi.GPIO as GPIO
                self._GPIO = GPIO
                # Nota: GPIO.setmode ja s'ha cridat des del controlador de servo
                GPIO.setup(cfg.fire_gpio_pin, GPIO.OUT, initial=GPIO.LOW)
                log.info(f"FireController: GPIO{cfg.fire_gpio_pin}")
            except Exception as exc:
                log.warning(f"FireController: no s'ha pogut inicialitzar: {exc}")
                cfg.fire_enabled = False

    # ------------------------------------------------------------------
    def update(self, on_target: bool) -> bool:
        """
        Crida cada frame.
        Retorna True si s'ha disparat en aquest frame.
        """
        if not self._cfg.fire_enabled:
            return False

        self._on_target_count = (self._on_target_count + 1) if on_target else 0

        if self._on_target_count >= self._cfg.fire_on_target_frames:
            disparat = self._disparar()
            self._on_target_count = 0
            return disparat
        return False

    # ------------------------------------------------------------------
    def _disparar(self) -> bool:
        now = time.time()
        if now - self._last_fire_time < self._cfg.fire_cooldown_s:
            return False
        self._last_fire_time = now

        if self._dry_run or self._GPIO is None:
            log.info("DISPARO SIMULAT")
            return True

        try:
            self._GPIO.output(self._cfg.fire_gpio_pin, self._GPIO.HIGH)
            time.sleep(self._cfg.fire_pulse_ms / 1000.0)
            self._GPIO.output(self._cfg.fire_gpio_pin, self._GPIO.LOW)
            log.info("DISPARO")
            return True
        except Exception as exc:
            log.error(f"Error disparo: {exc}")
            return False

    def reset(self):
        self._on_target_count = 0


# ===========================================================================
# AIM CONTROLLER — unifica PID + servos + disparo
# ===========================================================================

class AimController:
    """
    Interfície principal del mòdul de tir.

    Gestiona:
      - PID de pan (eix horitzontal)
      - PID de tilt (inclinació del canó, ±10°)
      - Servos (GPIO o PCA9685)
      - FireController

    Exemple d'ús:
        aim = AimController(cfg, modo="auto", dry_run=False)
        aim.actualitzar(error_x, error_y, on_target, area, w, h)
        aim.centre()
        aim.cleanup()
    """

    def __init__(self, cfg, modo: str = "auto", dry_run: bool = False):
        self._cfg = cfg
        self._servo = _crear_servo(modo, dry_run, cfg)
        self._fire  = FireController(cfg, dry_run)
        self._pid_pan  = PID(cfg.pid_pan_kp,  cfg.pid_pan_ki,  cfg.pid_pan_kd,
                             cfg.pid_max_output)
        self._pid_tilt = PID(cfg.pid_tilt_kp, cfg.pid_tilt_ki, cfg.pid_tilt_kd,
                             cfg.pid_max_output)
        self.pan_deg  = cfg.pan_center_deg
        self.tilt_deg = cfg.tilt_center_deg
        self._no_target_since: Optional[float] = None

        # Posiciona al centre
        self._servo.set_pan(self.pan_deg)
        self._servo.set_tilt(self.tilt_deg)
        time.sleep(0.3)
        log.info("AimController llest. PAN i TILT centrats.")

    # ------------------------------------------------------------------
    def actualitzar(self,
                    error_x: int,
                    error_y: int,
                    on_target: bool,
                    area_px: float,
                    frame_w: int,
                    frame_h: int) -> bool:
        """
        Actualitza la posició dels servos i comprova si cal disparar.

        Paràmetres:
          error_x  — diferència horitzontal (píxels) entre objectiu i centre frame
                     positiu = objectiu a la dreta
          error_y  — diferència vertical (píxels)
                     positiu = objectiu avall
          on_target — True si l'error és dins la zona morta
          area_px   — àrea del bounding box del globus (px²)
          frame_w/h — dimensions del frame

        Retorna True si s'ha produït un disparo.
        """
        cfg = self._cfg

        # Zona morta
        if abs(error_x) < cfg.dead_zone_px:
            error_x = 0
            self._pid_pan.reset()
        if abs(error_y) < cfg.dead_zone_px:
            error_y = 0
            self._pid_tilt.reset()

        # Correccions PID
        delta_pan  = self._pid_pan.update(error_x)
        delta_tilt = self._pid_tilt.update(error_y)

        # Offset de compensació càmera-canó
        tilt_offset = self._calcular_offset_tilt(area_px, frame_w, frame_h)

        # Nouvelles posicions amb límits físics
        self.pan_deg = max(cfg.pan_min_deg,
                          min(cfg.pan_max_deg,  self.pan_deg  + delta_pan))
        self.tilt_deg = max(cfg.tilt_min_deg,
                           min(cfg.tilt_max_deg, self.tilt_deg - delta_tilt + tilt_offset))

        self._servo.set_pan(self.pan_deg)
        self._servo.set_tilt(self.tilt_deg)
        self._no_target_since = None

        return self._fire.update(on_target)

    # ------------------------------------------------------------------
    def sense_objectiu(self):
        """
        Crida quan no es detecta cap globus.
        Després de `return_to_center_delay` segons, torna suaument al centre.
        """
        cfg = self._cfg
        now = time.time()
        if self._no_target_since is None:
            self._no_target_since = now
            return

        elapsed = now - self._no_target_since
        if elapsed > cfg.return_to_center_delay:
            self._pid_pan.reset()
            self._pid_tilt.reset()

            if cfg.smooth_return:
                step = cfg.return_speed_deg

                def suav(actual, centre):
                    if abs(actual - centre) > step:
                        return actual + step * (1 if centre > actual else -1)
                    return centre

                self.pan_deg  = suav(self.pan_deg,  cfg.pan_center_deg)
                self.tilt_deg = suav(self.tilt_deg, cfg.tilt_center_deg)
            else:
                self.pan_deg  = cfg.pan_center_deg
                self.tilt_deg = cfg.tilt_center_deg

            self._servo.set_pan(self.pan_deg)
            self._servo.set_tilt(self.tilt_deg)

        self._fire.reset()

    # ------------------------------------------------------------------
    def centre(self):
        """Torna immediatament al centre (crida al final)."""
        self.pan_deg  = self._cfg.pan_center_deg
        self.tilt_deg = self._cfg.tilt_center_deg
        self._servo.set_pan(self.pan_deg)
        self._servo.set_tilt(self.tilt_deg)

    # ------------------------------------------------------------------
    def cleanup(self):
        self.centre()
        time.sleep(0.3)
        self._servo.cleanup()
        log.info("AimController aturat.")

    # ------------------------------------------------------------------
    def _calcular_offset_tilt(self, area_px: float, frame_w: int, frame_h: int) -> float:
        cfg = self._cfg
        if area_px <= 0:
            return cfg.tilt_canon_offset_deg
        radi_px = math.sqrt(area_px / math.pi)
        focal_px = (frame_w / 2.0) / math.tan(math.radians(62.0) / 2.0)
        dist_m = max(0.5, (0.125 * focal_px) / max(radi_px, 1.0))
        return math.degrees(math.atan(cfg.canon_offset_cm / 100.0 / dist_m))
