# Robot Detector de Globus — Raspberry Pi 5

## Estructura del projecte

```
software/
├── main.py                     ← Punt d'entrada, bucle principal
├── config.py                   ← Tots els paràmetres del robot
├── estat.py                    ← Dataclass d'estat global
│
├── Visio d'elements/
│   ├── __init__.py
│   └── detector.py             ← Càmera CSI + inferència YOLOv8
│
├── Tir/
│   ├── __init__.py
│   └── apuntament.py           ← Servos PAN/TILT + PID + disparo
│
└── Desplaçament/
    ├── __init__.py
    └── motors.py               ← Motors de rodes + FSM
```

---

## Comportament

```
EXPLORANT → avança en línia recta fins que detecta un globus
    ↓ (globus detectat)
APROXIMANT → s'acosta (motors actius) tot apuntant amb els servos
    ↓ (àrea del globus > area_prou_a_prop)
APUNTANT → para els motors, afina l'apuntament amb PID
    ↓ (on_target durant N frames)
DISPARAT → dispara, cooldown, torna a EXPLORANT
```

---

## Paràmetres físics dels servos

| Servo | Funció | Centre | Rang |
|-------|--------|--------|------|
| PAN   | Gira tota la part superior (càmera + canó) | 90° | ±30° → [60°, 120°] |
| TILT  | Inclina **només el canó** amunt/avall | 90° | ±10° → [80°, 100°] |

---

## Connexions GPIO

### Servos
| Servo | Pin BCM |
|-------|---------|
| PAN   | GPIO 12 |
| TILT  | GPIO 13 |

### Motors de rodes (L298N o similar)
| Motor     | EN (PWM) | IN1 | IN2 |
|-----------|----------|-----|-----|
| Esquerre  | GPIO 24  | 23  | 25  |
| Dret      | GPIO 10  | 9   | 11  |

### Disparador
| Funció   | Pin BCM |
|----------|---------|
| Disparo  | GPIO 16 |

---

## Execució

```bash
# Arrenca el robot (mode normal)
python main.py

# Prova sense moure cap actuador
python main.py --dry-run

# Amb finestra de depuració (cal pantalla)
python main.py --preview

# Forçar GPIO directe
python main.py --modo gpio

# Forçar PCA9685 (I2C)
python main.py --modo pca9685

# Especificar model i confiança
python main.py --model /ruta/al/best.pt --conf 0.4
```

---

## Ajust de paràmetres

Tot es configura a `config.py`:

- **`velocitat_avancar`** (0–100): velocitat en mode EXPLORANT
- **`velocitat_apropar`** (0–100): velocitat en mode APROXIMANT
- **`area_prou_a_prop`** (px²): àrea del bounding box que indica "prou a prop"
- **`fire_on_target_frames`**: frames a punt abans de disparar
- **`dead_zone_px`**: zona morta de l'apuntament (píxels)
- **`pid_pan_kp/ki/kd`** i **`pid_tilt_kp/ki/kd`**: guanys del PID

---

## Dependències

```
ultralytics       # YOLOv8
opencv-python-headless
RPi.GPIO          # (o adafruit-circuitpython-pca9685 per a PCA9685)
numpy
```

S'instal·len automàticament si no hi són.
