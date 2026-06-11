"""
Tir/apuntament.py - Control de servos PAN/TILT i sistema de disparo
====================================================================
Responsabilitats:
  - Controladors de servo: DryRun / GPIO / PCA9685
  - Controlador PID per a pan i tilt
  - Classe FireController (disparar quan s'es a punt)
  - Classe AimController (unifica PID + servos + disparo)

Parametres fisics:
  - PAN  : gira tota la part superior +/-30 graus respecte al centre (60-120 graus)
  - TILT : inclina el cano +/-10 graus respecte al centre (80-100 graus)
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


def _deg_to_duty(deg: float, cfg) -> float:
    period_us = 1_000_000.0 / cfg.pwm_freq
    pulse_us  = cfg.servo_min_us + (deg / 180.0) * (cfg.servo_max_us - cfg.servo_min_us)
    return (pulse_us / period_us) * 100.0


def _deg_to_pca_ticks(deg: float, cfg) -> int:
    pulse_us = cfg.servo_min_us + (deg / 180.0) * (cfg.servo_max_us - cfg.servo_min_us)
    return max(0, min(4095, int((pulse_us / 20000.0) * 4096)))


class PID:
    def __init__(self, kp: float, ki: float, kd: float, max_output: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self._integral   = 0.0
        self._last_error = 0.0
        self._last_time  = time.time()

    def update(self, error: float) -> float:
        now  = time.time()
        dt   = max(now - self._last_time, 1e-3)
        self._integral   = max(-50.0, min(50.0, self._integral + error * dt))
        derivative       = (error - self._last_error) / dt
        output           = self.kp * error + self.ki * self._integral + self.kd * derivative
        self._last_error = error
        self._last_time  = now
        return max(-self.max_output, min(self.max_output, output))

    def reset(self):
        self._integral   = 0.0
        self._last_error = 0.0
        self._last_time  = time.time()


class _ServoBase:
    def set_pan(self, deg: float):  raise NotImplementedError
    def set_tilt(self, deg: float): raise NotImplementedError
    def cleanup(self): pass


class _DryRun(_ServoBase):
    def __init__(self):
        print("[Servos] Mode dry-run activat. Els servos no es mouraan.")
        log.info("Servos: MODE DRY-RUN")

    def set_pan(self, deg: float):
        log.debug(f"[DRY] PAN  -> {deg:.1f} graus")

    def set_tilt(self, deg: float):
        log.debug(f"[DRY] TILT -> {deg:.1f} graus")


class _GPIO(_ServoBase):
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
        self._pan.start(_deg_to_duty(cfg.pan_center_deg,  cfg))
        self._tilt.start(_deg_to_duty(cfg.tilt_center_deg, cfg))
        self._cfg = cfg
        print(f"[Servos] GPIO inicialitzat. PAN=GPIO{cfg.gpio_pan_pin}, TILT=GPIO{cfg.gpio_tilt_pin}")
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
        self._pca      = PCA9685(i2c, address=cfg.pca_i2c_address)
        self._pca.frequency = cfg.pwm_freq
        self._cfg      = cfg
        self._pan_ch   = cfg.pca_pan_channel
        self._tilt_ch  = cfg.pca_tilt_channel
        self.set_pan(cfg.pan_center_deg)
        self.set_tilt(cfg.tilt_center_deg)
        print(f"[Servos] PCA9685 inicialitzat. addr=0x{cfg.pca_i2c_address:02X} pan=ch{self._pan_ch} tilt=ch{self._tilt_ch}")
        log.info(f"PCA9685: addr=0x{cfg.pca_i2c_address:02X} pan=ch{self._pan_ch} tilt=ch{self._tilt_ch}")

    def _dc(self, ticks: int) -> int:
        return int(ticks * 65535 / 4095)

    def set_pan(self, deg: float):
        self._pca.channels[self._pan_ch].duty_cycle = self._dc(_deg_to_pca_ticks(deg, self._cfg))

    def set_tilt(self, deg: float):
        self._pca.channels[self._tilt_ch].duty_cycle = self._dc(_deg_to_pca_ticks(deg, self._cfg))

    def cleanup(self):
        self._pca.deinit()


def _crear_servo(modo: str, dry_run: bool, cfg) -> _ServoBase:
    if dry_run:
        return _DryRun()
    if modo == "gpio":
        return _GPIO(cfg)
    if modo == "pca9685":
        return _PCA9685(cfg)
    for cls in (_PCA9685, _GPIO):
        try:
            ctrl = cls(cfg)
            print(f"[Servos] Controlador auto-detectat: {cls.__name__}")
            log.info(f"Servo auto-detectat: {cls.__name__}")
            return ctrl
        except Exception:
            pass
    print("[Servos] No s'ha pogut connectar cap controlador. Activant dry-run.")
    log.warning("No s'ha pogut connectar cap controlador de servo. Activant dry-run.")
    return _DryRun()


class FireController:
    """
    Gestiona el disparo cridant motors.disparar().
    El disparo s'activa quan el robot porta fire_on_target_frames frames
    consecutius amb l'objectiu centrat.

    Rep una referencia al MotorController per poder cridar disparar().
    """

    def __init__(self, cfg, motors, dry_run: bool = False):
        self._cfg             = cfg
        self._motors          = motors
        self._dry_run         = dry_run
        self._on_target_count = 0
        self._last_fire_time  = 0.0

        if dry_run:
            print("[FireController] Inicialitzat en mode dry-run.")
        else:
            print(f"[FireController] Inicialitzat. Durada disparo: {cfg.fire_pulse_ms} ms")
        log.info("FireController inicialitzat.")

    def update(self, on_target: bool) -> bool:
        if not self._cfg.fire_enabled:
            return False
        self._on_target_count = (self._on_target_count + 1) if on_target else 0
        if self._on_target_count >= self._cfg.fire_on_target_frames:
            disparat = self._disparar()
            self._on_target_count = 0
            return disparat
        return False

    def _disparar(self) -> bool:
        now = time.time()
        if now - self._last_fire_time < self._cfg.fire_cooldown_s:
            return False
        self._last_fire_time = now

        if self._dry_run:
            print("[FireController] DISPARO SIMULAT")
            log.info("DISPARO SIMULAT")
            return True

        try:
            print("[FireController] DISPARO")
            log.info("DISPARO")
            self._motors.disparar(self._cfg.fire_pulse_ms / 1000.0)
            return True
        except Exception as exc:
            print(f"[FireController] Error disparo: {exc}")
            log.error(f"Error disparo: {exc}")
            return False

    def reset(self):
        self._on_target_count = 0


class AimController:
    """
    Interficie principal del modul de tir.

    Gestiona:
      - PID de pan (eix horitzontal)
      - PID de tilt (inclinacio del cano, +/-10 graus)
      - Servos (GPIO o PCA9685)
      - FireController (crida motors.disparar())

    Rep una referencia al MotorController per poder disparar.
    """

    def __init__(self, cfg, motors, modo: str = "auto", dry_run: bool = False):
        self._cfg  = cfg
        self._servo = _crear_servo(modo, dry_run, cfg)
        self._fire  = FireController(cfg, motors, dry_run)
        self._pid_pan  = PID(cfg.pid_pan_kp,  cfg.pid_pan_ki,  cfg.pid_pan_kd,  cfg.pid_max_output)
        self._pid_tilt = PID(cfg.pid_tilt_kp, cfg.pid_tilt_ki, cfg.pid_tilt_kd, cfg.pid_max_output)
        self.pan_deg   = cfg.pan_center_deg  + cfg.pan_offset_deg
        self.tilt_deg  = cfg.tilt_center_deg + cfg.tilt_offset_deg
        self._pan_enviat  = self.pan_deg
        self._tilt_enviat = self.tilt_deg
        self._no_target_since: Optional[float] = None

        self._servo.set_pan(self.pan_deg)
        self._servo.set_tilt(self.tilt_deg)
        time.sleep(0.3)
        print("[AimController] Servos centrats. PAN i TILT a posicio inicial.")
        log.info("AimController llest. PAN i TILT centrats.")

    def actualitzar(self,
                    error_x: int,
                    error_y: int,
                    on_target: bool,
                    area_px: float,
                    frame_w: int,
                    frame_h: int) -> bool:
        cfg = self._cfg

        if abs(error_x) < cfg.dead_zone_px:
            error_x = 0
            self._pid_pan.reset()
        if abs(error_y) < cfg.dead_zone_px:
            error_y = 0
            self._pid_tilt.reset()

        delta_pan   = self._pid_pan.update(error_x)
        delta_tilt  = self._pid_tilt.update(error_y)
        tilt_offset = self._calcular_offset_tilt(area_px, frame_w, frame_h)

        # Si el servo esta muntat al reves, inverteix la correccio
        if cfg.pan_invertit:
            delta_pan = -delta_pan
        if cfg.tilt_invertit:
            delta_tilt = -delta_tilt

        # Calcula el centre real tenint en compte l'offset de calibracio
        pan_centre_real  = cfg.pan_center_deg  + cfg.pan_offset_deg
        tilt_centre_real = cfg.tilt_center_deg + cfg.tilt_offset_deg

        nou_pan  = self.pan_deg  + delta_pan
        nou_tilt = self.tilt_deg - delta_tilt + tilt_offset

        # Limita sempre respecte als rangs fisics
        self.pan_deg  = max(cfg.pan_min_deg,  min(cfg.pan_max_deg,  nou_pan))
        self.tilt_deg = max(cfg.tilt_min_deg, min(cfg.tilt_max_deg, nou_tilt))

        # Filtre: nomes envia al servo si el canvi supera el minim configurat.
        # Aixo evita els "calfreds" causats pel jitter del PWM per software.
        min_canvi = cfg.servo_min_change_deg
        if abs(self.pan_deg - self._pan_enviat) >= min_canvi:
            self._servo.set_pan(self.pan_deg)
            self._pan_enviat = self.pan_deg
        if abs(self.tilt_deg - self._tilt_enviat) >= min_canvi:
            self._servo.set_tilt(self.tilt_deg)
            self._tilt_enviat = self.tilt_deg

        self._no_target_since = None

        return self._fire.update(on_target)

    def sense_objectiu(self):
        cfg = self._cfg
        now = time.time()
        if self._no_target_since is None:
            self._no_target_since = now
            return

        elapsed = now - self._no_target_since
        if elapsed > cfg.return_to_center_delay:
            self._pid_pan.reset()
            self._pid_tilt.reset()

            pan_centre_real  = cfg.pan_center_deg  + cfg.pan_offset_deg
            tilt_centre_real = cfg.tilt_center_deg + cfg.tilt_offset_deg

            if cfg.smooth_return:
                step = cfg.return_speed_deg

                def suav(actual, centre):
                    if abs(actual - centre) > step:
                        return actual + step * (1 if centre > actual else -1)
                    return centre

                self.pan_deg  = suav(self.pan_deg,  pan_centre_real)
                self.tilt_deg = suav(self.tilt_deg, tilt_centre_real)
            else:
                self.pan_deg  = pan_centre_real
                self.tilt_deg = tilt_centre_real

            min_canvi = cfg.servo_min_change_deg
            if abs(self.pan_deg - self._pan_enviat) >= min_canvi:
                self._servo.set_pan(self.pan_deg)
                self._pan_enviat = self.pan_deg
            if abs(self.tilt_deg - self._tilt_enviat) >= min_canvi:
                self._servo.set_tilt(self.tilt_deg)
                self._tilt_enviat = self.tilt_deg

        self._fire.reset()

    def centre(self):
        self.pan_deg  = self._cfg.pan_center_deg  + self._cfg.pan_offset_deg
        self.tilt_deg = self._cfg.tilt_center_deg + self._cfg.tilt_offset_deg
        self._servo.set_pan(self.pan_deg)
        self._servo.set_tilt(self.tilt_deg)

    def cleanup(self):
        self.centre()
        time.sleep(0.3)
        self._servo.cleanup()
        print("[AimController] Servos aturats.")
        log.info("AimController aturat.")

    def _calcular_offset_tilt(self, area_px: float, frame_w: int, frame_h: int) -> float:
        cfg = self._cfg
        if area_px <= 0:
            return cfg.tilt_canon_offset_deg
        radi_px  = math.sqrt(area_px / math.pi)
        focal_px = (frame_w / 2.0) / math.tan(math.radians(62.0) / 2.0)
        dist_m   = max(0.5, (0.125 * focal_px) / max(radi_px, 1.0))
        return math.degrees(math.atan(cfg.canon_offset_cm / 100.0 / dist_m))
