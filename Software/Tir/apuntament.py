"""
Tir/apuntament.py - Control de servos PAN/TILT i sistema de disparo
====================================================================
Responsabilitats:
  - Controlador de servo PCA9685 (I2C) i mode DryRun
  - Controlador PID per a pan i tilt
  - Classe FireController (disparar quan s'es a punt)
  - Classe AimController (unifica PID + servos + disparo)

Parametres fisics:
  - PAN  : gira tota la part superior +/-30 graus respecte al centre (60-120 graus)
  - TILT : inclina el cano, rang 55-90 graus

La camera esta a la mateixa alcada que el cano pero desplacada uns
3-5 cm a la dreta (camera_offset_horitzontal_cm a config.py). Aixo es
corregeix afegint un offset constant a l'angle PAN, ja que el cano
esta lleugerament a l'esquerra del que veu la camera.
"""

import sys
import time
import math
import logging
import threading
from typing import Optional

log = logging.getLogger("robot.tir")


def _install(pkgs):
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", *pkgs, "-q"])


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
        log.info("Servos: MODE DRY-RUN")

    def set_pan(self, deg: float):
        log.debug(f"[DRY] PAN  -> {deg:.1f} graus")

    def set_tilt(self, deg: float):
        log.debug(f"[DRY] TILT -> {deg:.1f} graus")


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
    if modo == "pca9685":
        try:
            return _PCA9685(cfg)
        except Exception as exc:
            log.warning(f"PCA9685 no disponible: {exc}")
            return _DryRun()
    return _DryRun()


class FireController:
    """
    Gestiona el disparo cridant motors.disparar().
    El disparo s'activa immediatament quan el punt d'apuntament del cano
    cau dins el bounding box del globus (on_target=True), sense esperar
    frames addicionals. Nomes es respecta el cooldown entre disparos.

    Rep una referencia al MotorController per poder cridar disparar().
    """

    def __init__(self, cfg, motors, dry_run: bool = False):
        self._cfg            = cfg
        self._motors         = motors
        self._dry_run        = dry_run
        self._last_fire_time = 0.0

    def update(self, on_target: bool) -> bool:
        if not self._cfg.fire_enabled:
            return False
        if on_target:
            return self._disparar()
        return False

    def _disparar(self) -> bool:
        now = time.time()
        if now - self._last_fire_time < self._cfg.fire_cooldown_s:
            return False
        self._last_fire_time = now

        if self._dry_run:
            print("DISPARO SIMULAT")
            return True

        try:
            print("DISPARO")
            self._motors.disparar(self._cfg.fire_pulse_ms / 1000.0)
            return True
        except Exception as exc:
            print(f"DISPARO - ERROR: {exc}")
            return False

    def reset(self):
        pass


class AimController:
    """
    Interficie principal del modul de tir.

    Gestiona:
      - PID de pan (eix horitzontal)
      - PID de tilt (inclinacio del cano)
      - Servo PCA9685
      - FireController (crida motors.disparar())
      - Compensacio de l'offset horitzontal entre camera i cano

    Rep una referencia al MotorController per poder disparar.
    """

    def __init__(self, cfg, motors, modo: str = "pca9685", dry_run: bool = False):
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
        log.info("AimController llest. PAN i TILT centrats.")

        # Thread de swipe continu durant EXPLORANT.
        # S'executa a alta freqüencia (independent del bucle principal /
        # FPS de la camera) per fer el moviment del servo PAN fluid.
        self._swipe_lock    = threading.Lock()
        self._swipe_actiu   = False
        self._swipe_direccio = 1
        self._swipe_thread  = threading.Thread(target=self._bucle_swipe, daemon=True)
        self._swipe_thread.start()

    def actualitzar(self,
                    error_x: int,
                    error_y: int,
                    on_target: bool,
                    area_px: float,
                    frame_w: int,
                    frame_h: int,
                    canon_offset_px: float = 0.0) -> bool:
        """
        Actualitza la posicio dels servos.

        canon_offset_px: correccio horitzontal en pixels per compensar que
        el cano esta desplacat respecte la camera (positiu = cano a la
        dreta de la camera, negatiu = a l'esquerra).
        """
        cfg = self._cfg

        # Quan s'esta apuntant a un globus, el swipe continu no ha d'interferir
        self.aturar_swipe()

        # Aplica la correccio de l'offset camera-cano abans de calcular l'error
        error_x = error_x - canon_offset_px

        if abs(error_x) < cfg.dead_zone_px:
            error_x = 0
            self._pid_pan.reset()
        if abs(error_y) < cfg.dead_zone_px:
            error_y = 0
            self._pid_tilt.reset()

        delta_pan  = self._pid_pan.update(error_x)
        delta_tilt = self._pid_tilt.update(error_y)

        # Si el servo esta muntat al reves, inverteix la correccio
        if cfg.pan_invertit:
            delta_pan = -delta_pan
        if cfg.tilt_invertit:
            delta_tilt = -delta_tilt

        nou_pan  = self.pan_deg  + delta_pan
        nou_tilt = self.tilt_deg - delta_tilt

        # Limita la velocitat de moviment del servo (graus per frame)
        nou_pan  = self._limitar_velocitat(self.pan_deg,  nou_pan,  cfg.servo_vel_max_deg)
        nou_tilt = self._limitar_velocitat(self.tilt_deg, nou_tilt, cfg.servo_vel_max_deg)

        # Limita sempre respecte als rangs fisics
        pan_limitat  = max(cfg.pan_min_deg,  min(cfg.pan_max_deg,  nou_pan))
        tilt_limitat = max(cfg.tilt_min_deg, min(cfg.tilt_max_deg, nou_tilt))

        # Si el servo ha tocat un limit, reseteja l'integral del PID
        if pan_limitat != nou_pan:
            self._pid_pan.reset()
        if tilt_limitat != nou_tilt:
            self._pid_tilt.reset()

        self.pan_deg  = pan_limitat
        self.tilt_deg = tilt_limitat

        self._enviar_si_cal()
        self._no_target_since = None

        return self._fire.update(on_target)

    def iniciar_swipe(self):
        """Activa el swipe continu del servo PAN (estat EXPLORANT)."""
        with self._swipe_lock:
            self._swipe_actiu = True

    def aturar_swipe(self):
        """Atura el swipe continu del servo PAN."""
        with self._swipe_lock:
            self._swipe_actiu = False

    def _bucle_swipe(self):
        """
        Thread que mou el servo PAN d'esquerra a dreta i viceversa de
        forma continua i fluida, a alta freqüencia (independent del FPS
        de la camera). Nomes actua quan _swipe_actiu es True.
        """
        cfg = self._cfg
        periode = 1.0 / 30.0   # actualitza el servo a ~30 Hz
        last_time = time.time()

        while True:
            time.sleep(periode)

            with self._swipe_lock:
                actiu = self._swipe_actiu

            ara = time.time()
            dt = ara - last_time
            last_time = ara

            if not actiu:
                continue

            pan_dreta    = cfg.pan_max_deg
            pan_esquerra = cfg.pan_min_deg

            increment = cfg.cerca_vel_deg_s * dt
            nou_pan = self.pan_deg + self._swipe_direccio * increment

            if nou_pan >= pan_dreta:
                nou_pan = pan_dreta
                self._swipe_direccio = -1
            elif nou_pan <= pan_esquerra:
                nou_pan = pan_esquerra
                self._swipe_direccio = 1

            self.pan_deg = nou_pan

            # Manté el tilt al centre mentre s'explora
            tilt_centre = cfg.tilt_center_deg + cfg.tilt_offset_deg
            nou_tilt = self._limitar_velocitat(self.tilt_deg, tilt_centre, cfg.servo_vel_max_deg)
            self.tilt_deg = max(cfg.tilt_min_deg, min(cfg.tilt_max_deg, nou_tilt))

            self._enviar_si_cal()

    def sense_objectiu(self):
        """Marca que no hi ha objectiu (resetejar comptadors de disparo)."""
        if self._no_target_since is None:
            self._no_target_since = time.time()
        self._fire.reset()
        self._pid_pan.reset()
        self._pid_tilt.reset()

    def centre(self):
        self.pan_deg  = self._cfg.pan_center_deg  + self._cfg.pan_offset_deg
        self.tilt_deg = self._cfg.tilt_center_deg + self._cfg.tilt_offset_deg
        self._servo.set_pan(self.pan_deg)
        self._servo.set_tilt(self.tilt_deg)
        self._pan_enviat  = self.pan_deg
        self._tilt_enviat = self.tilt_deg

    def cleanup(self):
        self.aturar_swipe()
        self.centre()
        time.sleep(0.3)
        self._servo.cleanup()
        log.info("AimController aturat.")

    def _limitar_velocitat(self, actual: float, desitjat: float, vel_max: float) -> float:
        """Limita el canvi entre actual i desitjat a vel_max graus."""
        diff = desitjat - actual
        if diff > vel_max:
            return actual + vel_max
        if diff < -vel_max:
            return actual - vel_max
        return desitjat

    def _enviar_si_cal(self):
        """Envia la nova posicio al servo nomes si el canvi supera el minim configurat."""
        cfg = self._cfg
        min_canvi = cfg.servo_min_change_deg
        if abs(self.pan_deg - self._pan_enviat) >= min_canvi:
            self._servo.set_pan(self.pan_deg)
            self._pan_enviat = self.pan_deg
        if abs(self.tilt_deg - self._tilt_enviat) >= min_canvi:
            self._servo.set_tilt(self.tilt_deg)
            self._tilt_enviat = self.tilt_deg
