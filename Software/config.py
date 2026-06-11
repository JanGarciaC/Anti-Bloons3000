"""
config.py - Configuracio central del robot
==========================================
Tots els parametres del robot en un sol lloc.
"""

from dataclasses import dataclass


@dataclass
class Config:
    # Model ONNX (YOLOv8 exportat)
    # Exporta amb: yolo export model=best.pt format=onnx imgsz=320 simplify=True
    model_path:       str   = ""        # buit = cerca automatica (best.onnx)
    conf_threshold:   float = 0.35
    iou_threshold:    float = 0.45

    # Camera
    camera_index:     int   = 0
    frame_width:      int   = 640
    frame_height:     int   = 480
    fps_target:       int   = 30

    # Servos: rangs fisics
    # PAN : gira tota la part superior (camera + cano) dreta/esquerra
    #       centre=90, limits +/-30 -> [60, 120]
    pan_min_deg:      float = 60.0
    pan_max_deg:      float = 120.0
    pan_center_deg:   float = 90.0
    # Si el servo gira al reves del que toca, posa pan_invertit = True
    pan_invertit:     bool  = False
    # Si el servo esta centrat pero desplacat mecanicament, ajusta aquest valor
    # Per exemple: si el centre real es a 95 graus, posa pan_offset_deg = 5.0
    pan_offset_deg:   float = 0.0

    # TILT: inclina nomes el cano amunt/avall
    #       centre=90, limits +/-10 -> [80, 100]
    tilt_min_deg:     float = 80.0
    tilt_max_deg:     float = 100.0
    tilt_center_deg:  float = 90.0
    tilt_invertit:    bool  = False
    tilt_offset_deg:  float = 0.0

    # Servos: PWM
    pwm_freq:         int   = 50
    servo_min_us:     float = 500.0
    servo_max_us:     float = 2500.0

    # Servos: GPIO directe
    gpio_pan_pin:     int   = 12        # BCM (PWM hardware)
    gpio_tilt_pin:    int   = 13        # BCM (PWM hardware)

    # Servos: PCA9685 (I2C)
    pca_i2c_address:  int   = 0x40
    pca_pan_channel:  int   = 0
    pca_tilt_channel: int   = 1

    # Control PID (apuntament)
    pid_pan_kp:       float = 0.035
    pid_pan_ki:       float = 0.0005
    pid_pan_kd:       float = 0.008
    pid_tilt_kp:      float = 0.030
    pid_tilt_ki:      float = 0.0005
    pid_tilt_kd:      float = 0.006
    pid_max_output:   float = 5.0
    dead_zone_px:     int   = 25      # zona morta mes gran per reduir moviment innecessari

    # Filtre de moviment minim del servo
    # El servo nomes es mou si el canvi es superior a aquest valor (graus).
    # Augmenta si els servos tenen "calfreds" (recomanat: 0.4 - 1.0)
    servo_min_change_deg: float = 0.5

    # Offset camera-cano
    canon_offset_cm:       float = 15.0
    tilt_canon_offset_deg: float = 2.9

    # Comportament servo sense target
    return_to_center_delay: float = 2.0
    smooth_return:          bool  = True
    return_speed_deg:       float = 2.0

    # Motors DC via transistors NPN (BC547, 2N2222 o similar)
    # Un pin GPIO per motor: HIGH = engegat, LOW = aturat
    motor_esq_pin:     int   = 17      # GPIO BCM - motor roda esquerra
    motor_dre_pin:     int   = 27      # GPIO BCM - motor roda dreta
    motor_disparo_pin: int   = 22      # GPIO BCM - motor disparo

    # Velocitat (no s'usa amb transistors simples, es manté per compatibilitat)
    velocitat_avancar: int  = 100
    velocitat_apropar: int  = 100

    # Sistema de disparo
    fire_enabled:          bool  = True
    fire_on_target_frames: int   = 5   # frames a punt abans de disparar
    fire_pulse_ms:         float = 500.0  # durada del pols del motor de disparo
    fire_cooldown_s:       float = 2.0

    # Area minima del globus (px quadrats) per considerar-lo prou a prop i parar
    area_prou_a_prop: float = 18000.0


CFG = Config()
