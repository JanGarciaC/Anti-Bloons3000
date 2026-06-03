"""
desplacament/motors.py — Control de rodes i màquina d'estats del robot
=======================================================================
Responsabilitats:
  - MotorController: gestiona els dos motors de les rodes via GPIO + PWM
  - RobotFSM: màquina d'estats finits que coordina els comportaments:

      ┌─────────────┐  globus detectat   ┌─────────────┐
      │  EXPLORANT  │ ─────────────────► │ APROXIMANT  │
      │ (avança en  │                    │ (s'acosta   │
      │  línia recta│ ◄─────────────────  │  al globus) │
      └─────────────┘  globus perdut     └──────┬──────┘
                                                │ prou a prop
                                                ▼
                                         ┌─────────────┐
                                         │  APUNTANT   │
                                         │ (para motors│
                                         │ i apunta)   │
                                         └──────┬──────┘
                                                │ on_target
                                                ▼
                                         ┌─────────────┐
                                         │  DISPARAT   │
                                         │ (disparo    │
                                         │ + cooldown) │
                                         └──────┬──────┘
                                                │ cooldown acabat
                                                ▼
                                         torna a EXPLORANT

Connexions GPIO (L298N o equivalent):
  Motor esquerre: EN=24(PWM), IN1=23, IN2=25
  Motor dret:     EN=10(PWM), IN1=9,  IN2=11

Ús:
    from desplacament.motors import MotorController, RobotFSM
    motors = MotorController(cfg)
    fsm    = RobotFSM(cfg, motors)
    fsm.actualitzar(objectiu_detectat, prou_a_prop, on_target, disparat)
    motors.parar()
    motors.cleanup()
"""

import sys
import time
import logging
from typing import Optional

log = logging.getLogger("robot.desplacament")


def _install(pkgs):
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", *pkgs, "-q"])


# ===========================================================================
# STATES FSM
# ===========================================================================
EXPLORANT  = "EXPLORANT"
APROXIMANT = "APROXIMANT"
APUNTANT   = "APUNTANT"
DISPARAT   = "DISPARAT"


# ===========================================================================
# MOTOR CONTROLLER
# ===========================================================================

class MotorController:
    """
    Controla els dos motors de rodes via un pont H (L298N o similar).

    Cada motor té:
      - pin EN  (PWM) : velocitat
      - pin IN1, IN2  : direcció (01=endavant, 10=enrere, 00=parar)
    """

    def __init__(self, cfg, dry_run: bool = False):
        self._cfg = cfg
        self._dry_run = dry_run
        self._GPIO = None
        self._pwm_esq = None
        self._pwm_dre = None

        if dry_run:
            log.info("MotorController: MODE DRY-RUN")
            return

        try:
            import RPi.GPIO as GPIO
        except ImportError:
            _install(["RPi.GPIO"]); import RPi.GPIO as GPIO

        self._GPIO = GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Configura pins motor esquerre
        for pin in (cfg.motor_esq_en, cfg.motor_esq_in1, cfg.motor_esq_in2):
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

        # Configura pins motor dret
        for pin in (cfg.motor_dre_en, cfg.motor_dre_in1, cfg.motor_dre_in2):
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

        self._pwm_esq = GPIO.PWM(cfg.motor_esq_en, cfg.motors_pwm_freq)
        self._pwm_dre = GPIO.PWM(cfg.motor_dre_en, cfg.motors_pwm_freq)
        self._pwm_esq.start(0)
        self._pwm_dre.start(0)

        log.info(
            f"MotorController: ESQ(EN={cfg.motor_esq_en}, IN1={cfg.motor_esq_in1}, "
            f"IN2={cfg.motor_esq_in2}) | "
            f"DRE(EN={cfg.motor_dre_en}, IN1={cfg.motor_dre_in1}, IN2={cfg.motor_dre_in2})"
        )

    # ------------------------------------------------------------------
    def _set_motor(self, en_pwm, in1_pin, in2_pin, velocitat: int, endavant: bool):
        """
        Aplica velocitat i direcció a un motor.
        velocitat: 0–100 (duty cycle)
        endavant:  True = endavant, False = enrere
        """
        if self._dry_run or self._GPIO is None:
            return
        GPIO = self._GPIO
        if velocitat == 0:
            GPIO.output(in1_pin, GPIO.LOW)
            GPIO.output(in2_pin, GPIO.LOW)
            en_pwm.ChangeDutyCycle(0)
        elif endavant:
            GPIO.output(in1_pin, GPIO.HIGH)
            GPIO.output(in2_pin, GPIO.LOW)
            en_pwm.ChangeDutyCycle(velocitat)
        else:
            GPIO.output(in1_pin, GPIO.LOW)
            GPIO.output(in2_pin, GPIO.HIGH)
            en_pwm.ChangeDutyCycle(velocitat)

    # ------------------------------------------------------------------
    def avancar(self, velocitat: Optional[int] = None):
        """Tots dos motors endavant a la velocitat indicada."""
        v = velocitat if velocitat is not None else self._cfg.velocitat_avancar
        v = max(0, min(100, v))
        cfg = self._cfg
        if self._dry_run:
            log.debug(f"[DRY] AVANCAR v={v}%")
            return
        self._set_motor(self._pwm_esq, cfg.motor_esq_in1, cfg.motor_esq_in2, v, True)
        self._set_motor(self._pwm_dre, cfg.motor_dre_in1, cfg.motor_dre_in2, v, True)

    def parar(self):
        """Atura ambdós motors."""
        if self._dry_run:
            log.debug("[DRY] PARAR")
            return
        cfg = self._cfg
        self._set_motor(self._pwm_esq, cfg.motor_esq_in1, cfg.motor_esq_in2, 0, True)
        self._set_motor(self._pwm_dre, cfg.motor_dre_in1, cfg.motor_dre_in2, 0, True)

    def enrere(self, velocitat: Optional[int] = None):
        """Tots dos motors enrere (per si cal recular)."""
        v = velocitat if velocitat is not None else self._cfg.velocitat_avancar
        v = max(0, min(100, v))
        cfg = self._cfg
        if self._dry_run:
            log.debug(f"[DRY] ENRERE v={v}%")
            return
        self._set_motor(self._pwm_esq, cfg.motor_esq_in1, cfg.motor_esq_in2, v, False)
        self._set_motor(self._pwm_dre, cfg.motor_dre_in1, cfg.motor_dre_in2, v, False)

    def cleanup(self):
        """Atura i allibera GPIO."""
        self.parar()
        if self._pwm_esq:
            self._pwm_esq.stop()
        if self._pwm_dre:
            self._pwm_dre.stop()
        if self._GPIO:
            # Nota: GPIO.cleanup() es crida globalment des del main
            pass
        log.info("MotorController aturat.")


# ===========================================================================
# MÀQUINA D'ESTATS FINITS (FSM)
# ===========================================================================

class RobotFSM:
    """
    Coordina el comportament global del robot:

      EXPLORANT  → avança en línia recta fins que detecta un globus
      APROXIMANT → s'acosta al globus (motors + servos actius)
      APUNTANT   → para els motors, apunta amb els servos
      DISPARAT   → ha disparat, cooldown, torna a explorar

    Ús per frame:
        disparat = fsm.actualitzar(
            objectiu_detectat = objectiu is not None,
            prou_a_prop       = area > cfg.area_prou_a_prop,
            on_target         = aim.on_target,
            disparo_produit   = aim_ret_val,   # booleà de AimController.actualitzar()
        )
    """

    COOLDOWN_DISPARO_S = 1.5  # temps a estat DISPARAT abans de tornar a explorar

    def __init__(self, cfg, motors: MotorController):
        self._cfg    = cfg
        self._motors = motors
        self.fase    = EXPLORANT
        self._disparo_time: Optional[float] = None
        log.info(f"RobotFSM inicialitzat. Estat inicial: {self.fase}")

    # ------------------------------------------------------------------
    def actualitzar(self,
                    objectiu_detectat: bool,
                    prou_a_prop: bool,
                    on_target: bool,
                    disparo_produit: bool) -> str:
        """
        Actualitza la FSM i els motors.
        Retorna el nom de l'estat actual.
        """
        fase_anterior = self.fase

        # ── EXPLORANT ────────────────────────────────────────────────────
        if self.fase == EXPLORANT:
            if objectiu_detectat:
                self._canviar_fase(APROXIMANT)
            else:
                self._motors.avancar(self._cfg.velocitat_avancar)

        # ── APROXIMANT ───────────────────────────────────────────────────
        elif self.fase == APROXIMANT:
            if not objectiu_detectat:
                # Hem perdut el globus → torna a explorar
                self._canviar_fase(EXPLORANT)
            elif prou_a_prop:
                # Prou a prop → para i apunta
                self._motors.parar()
                self._canviar_fase(APUNTANT)
            else:
                # Segueix avançant cap al globus
                self._motors.avancar(self._cfg.velocitat_apropar)

        # ── APUNTANT ─────────────────────────────────────────────────────
        elif self.fase == APUNTANT:
            if not objectiu_detectat:
                # Hem perdut el globus → torna a explorar
                self._canviar_fase(EXPLORANT)
            elif disparo_produit:
                self._canviar_fase(DISPARAT)
                self._disparo_time = time.time()
            # si no, AimController segueix ajustant els servos

        # ── DISPARAT ─────────────────────────────────────────────────────
        elif self.fase == DISPARAT:
            self._motors.parar()
            if time.time() - self._disparo_time >= self.COOLDOWN_DISPARO_S:
                log.info("Cooldown acabat. Tornant a explorar.")
                self._canviar_fase(EXPLORANT)

        if self.fase != fase_anterior:
            log.info(f"FSM: {fase_anterior} → {self.fase}")

        return self.fase

    # ------------------------------------------------------------------
    def _canviar_fase(self, nova: str):
        self.fase = nova
        if nova == EXPLORANT:
            # Torna als servos al centre (ho fa AimController)
            pass
        elif nova == APROXIMANT:
            pass
        elif nova == APUNTANT:
            self._motors.parar()
        elif nova == DISPARAT:
            self._motors.parar()
