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
    model_path:       str   = ""
    conf_threshold:   float = 0.35
    iou_threshold:    float = 0.45

    # Camera
    camera_index:     int   = 0
    frame_width:      int   = 480
    frame_height:     int   = 320
    fps_target:       int   = 30

    # Salta frames entre inferencies per augmentar el FPS efectiu.
    # 1 = processa tots els frames, 2 = processa 1 de cada 2, etc.
    inferencia_cada_n_frames: int = 1

    # Offset horitzontal camera-cano (cm, positiu = cano a l'esquerra de la camera)
    # La camera esta 3-5 cm a la dreta del cano, per tant el cano apunta
    # lleugerament a l'esquerra del que veu la camera.
    # Ajusta aquest valor amb la distancia real mesurada.
    camera_offset_horitzontal_cm: float = 4.0

    # Servos: rangs fisics
    # PAN : gira tota la part superior (camera + cano) dreta/esquerra
    #       centre=90, limits +/-30 -> [60, 120]
    pan_min_deg:      float = 60.0
    pan_max_deg:      float = 120.0
    pan_center_deg:   float = 90.0
    pan_invertit:     bool  = False
    pan_offset_deg:   float = 0.0

    # TILT: inclina nomes el cano amunt/avall, limits 55-90 graus
    tilt_min_deg:     float = 55.0
    tilt_max_deg:     float = 90.0
    tilt_center_deg:  float = 72.5
    tilt_invertit:    bool  = False
    tilt_offset_deg:  float = 0.0

    # Servos via PCA9685 (I2C)
    # Canal 0 -> servo gir horitzontal (PAN)
    # Canal 1 -> servo inclinacio cano (TILT)
    pwm_freq:         int   = 50
    servo_min_us:     float = 500.0
    servo_max_us:     float = 2500.0
    pca_i2c_address:  int   = 0x40
    pca_pan_channel:  int   = 0
    pca_tilt_channel: int   = 1

    # Velocitat maxima de moviment dels servos (graus per frame)
    # Redueix aquest valor per fer els servos mes lents i suaus
    servo_vel_max_deg: float = 2.0

    # Control PID (apuntament)
    pid_pan_kp:       float = 0.025
    pid_pan_ki:       float = 0.0003
    pid_pan_kd:       float = 0.006
    pid_tilt_kp:      float = 0.020
    pid_tilt_ki:      float = 0.0003
    pid_tilt_kd:      float = 0.005
    pid_max_output:   float = 3.0
    dead_zone_px:     int   = 25

    # Filtre de moviment minim del servo (graus)
    servo_min_change_deg: float = 0.5

    # Comportament servo sense target
    return_to_center_delay: float = 1.5
    return_speed_deg:       float = 1.0   # graus per frame al tornar al centre

    # Cerca de globus (comportament EXPLORANT)
    # El robot avanca contínuament a poc a poc mentre el servo PAN fa
    # un swipe continu (dreta -> esquerra -> dreta...) sense parar.
    # La velocitat del swipe es en graus per segon (independent del FPS).
    cerca_vel_deg_s: float = 25.0   # graus per segon del swipe del servo PAN

    # Motors DC via transistors NPN
    motor_esq_pin:     int   = 17
    motor_dre_pin:     int   = 27
    motor_disparo_pin: int   = 22

    # Velocitat de les rodes (0-100). 100 = sempre engegat (maxima velocitat).
    # Valors mes baixos fan que el motor s'engegui i s'apagui rapidament
    # (PWM per software) per simular una velocitat mes lenta.
    velocitat_rodes:     int = 50
    motors_pwm_freq_hz:  float = 50.0   # frequencia del PWM per software (Hz)

    # Sistema de disparo
    fire_enabled:          bool  = True
    fire_on_target_frames: int   = 5
    fire_pulse_ms:         float = 500.0
    fire_cooldown_s:       float = 2.0

    # Area minima del globus (px quadrats) per considerar-lo prou a prop i parar
    area_prou_a_prop: float = 18000.0


CFG = Config()
