"""
config.py — Configuració central del robot
==========================================
Tots els paràmetres del robot en un sol lloc.
Importa aquest mòdul des de qualsevol altre mòdul del projecte.
"""

from dataclasses import dataclass


@dataclass
class Config:
    # ── Model YOLO ──────────────────────────────────────────────────────────
    model_path:       str   = ""        # buit = cerca automàtica
    conf_threshold:   float = 0.35
    iou_threshold:    float = 0.45

    # ── Càmera ──────────────────────────────────────────────────────────────
    camera_index:     int   = 0
    frame_width:      int   = 640
    frame_height:     int   = 480
    fps_target:       int   = 30

    # ── Servos: rangs físics ─────────────────────────────────────────────────
    # PAN: gira tota la part superior (càmera + canó) dreta/esquerra
    #      centre=90°, límits ±30° → [60°, 120°]
    pan_min_deg:      float = 60.0
    pan_max_deg:      float = 120.0
    pan_center_deg:   float = 90.0

    # TILT: inclina només el canó amunt/avall
    #       centre=90°, límits ±10° → [80°, 100°]
    tilt_min_deg:     float = 80.0
    tilt_max_deg:     float = 100.0
    tilt_center_deg:  float = 90.0

    # ── Servos: PWM ──────────────────────────────────────────────────────────
    pwm_freq:         int   = 50
    servo_min_us:     float = 500.0
    servo_max_us:     float = 2500.0

    # ── GPIO (mode directe) ──────────────────────────────────────────────────
    gpio_pan_pin:     int   = 12        # BCM (PWM hardware)
    gpio_tilt_pin:    int   = 13        # BCM (PWM hardware)

    # ── PCA9685 (mode I2C) ───────────────────────────────────────────────────
    pca_i2c_address:  int   = 0x40
    pca_pan_channel:  int   = 0
    pca_tilt_channel: int   = 1

    # ── Control PID (apuntament) ─────────────────────────────────────────────
    pid_pan_kp:       float = 0.055
    pid_pan_ki:       float = 0.001
    pid_pan_kd:       float = 0.012
    pid_tilt_kp:      float = 0.045
    pid_tilt_ki:      float = 0.001
    pid_tilt_kd:      float = 0.010
    pid_max_output:   float = 8.0       # màxim graus de correcció per frame
    dead_zone_px:     int   = 15        # píxels d'error mínim per moure's

    # ── Offset càmera-canó ───────────────────────────────────────────────────
    canon_offset_cm:       float = 15.0
    tilt_canon_offset_deg: float = 2.9

    # ── Comportament servo sense target ──────────────────────────────────────
    return_to_center_delay: float = 2.0
    smooth_return:          bool  = True
    return_speed_deg:       float = 2.0

    # ── Sistema de disparo ───────────────────────────────────────────────────
    fire_enabled:          bool  = True
    fire_gpio_pin:         int   = 16
    fire_on_target_frames: int   = 5
    fire_pulse_ms:         float = 150.0
    fire_cooldown_s:       float = 2.0   # mínim temps entre disparos

    # ── Motors de rodes ──────────────────────────────────────────────────────
    # Motor esquerre
    motor_esq_en:          int   = 24   # pin ENABLE (PWM)
    motor_esq_in1:         int   = 23   # direcció
    motor_esq_in2:         int   = 25   # direcció

    # Motor dret
    motor_dre_en:          int   = 10   # pin ENABLE (PWM)
    motor_dre_in1:         int   = 9    # direcció
    motor_dre_in2:         int   = 11   # direcció

    # Velocitat (0–100, duty cycle PWM)
    velocitat_avancar:     int   = 40   # velocitat crucero en línia recta
    velocitat_apropar:     int   = 30   # velocitat quan s'acosta al globus
    motors_pwm_freq:       int   = 1000

    # Àrea mínima del globus (px²) per considerar-lo "a prop" i parar
    area_prou_a_prop:      float = 18000.0

    # ── Estat de l'FSM ───────────────────────────────────────────────────────
    # (no modifiques; és l'estat inicial)
    # EXPLORANT → APROXIMANT → APUNTANT → DISPARAT


CFG = Config()
