# Anti-Bloons 3000

L'Anti-Bloons 3000 és un robot que detecta, restreja i elimina globus a distància. Inspirant-nos en la saga de jocs "Bloons TD", volem materialitzar un autòmat capaç de disparar projectils calculant la posició dels objectius (en el nostre cas, globus d'aire).

![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54) ![Raspberry Pi](https://img.shields.io/badge/-Raspberry_Pi-C51A4A?style=for-the-badge&logo=Raspberry-Pi)

## Autors

- [Jan Garcia Comas](https://www.github.com/JanGarciaC) - Software Lead
- [Aarón Móstiga Móstiga](https://www.github.com/NNIU1708231) - 3D Parts & Mechanical Lead
- [Gerard Casanovas Urpí](https://www.github.com/NIU1708307) - Hardware Lead
- [Lluc Aymerich Medina](https://www.github.com/Insaly) - Testing & Validation Lead

# Components

## Model 3D

A continuació trobem les diferents parts del model inicial.

![Model3D](https://media.discordapp.net/attachments/1164216965720719383/1501577440626081853/02ea1c85-2ae6-4305-86d0-231387ccf630.png?ex=69fc9477&is=69fb42f7&hm=73c554943341116c73c70352137a176cd4b0691f85d27b74a73cc8178472a094&=&format=webp&quality=lossless&width=964&height=930)

## Hardware 

Els components de l'Anti-Bloons son "prou simples", utilitzant una Raspberry Pi 4 com a unitat de processament central.

### Components necessaris
| Component | Model | Unitats |
|-----------|-------|-------|
| <img src="https://raw.githubusercontent.com/JanGarciaC/Anti-Bloons3000/main/Hardware/Components/Imatges/raspberrypi4.jpg" width="75"> | [Raspberry Pi 4 Model B](https://tienda.bricogeek.com/placas-raspberry-pi/1330-raspberry-pi-4-model-b-4-gb.html) | 1 |
| <img src="https://raw.githubusercontent.com/JanGarciaC/Anti-Bloons3000/main/Hardware/Components/Imatges/raspycamv2-8mp.jpg" width="75"> | [Cámara Raspberry Pi v2 - 8 Megapixels](https://tienda.bricogeek.com/sensores-imagen/822-camara-raspberry-pi-v2-8-megapixels.html) | 1 |
| <img src="https://raw.githubusercontent.com/JanGarciaC/Anti-Bloons3000/main/Hardware/Components/Imatges/rplidar-c1-ip54.jpg" width="75"> | [RPLIDAR-C1 360 grados 12 metros (IP54)](https://tienda.bricogeek.com/sensores-distancia/1943-rplidar-c1-360-grados-12-metros-ip54.html) | 1 |
| <img src="https://raw.githubusercontent.com/JanGarciaC/Anti-Bloons3000/main/Hardware/Components/Imatges/motor30-1.jpg" width="75"> | [Motor micro metal 30:1 HP con eje extendido](https://tienda.bricogeek.com/motores-dc/1007-motor-micro-metal-30-1-hp-con-eje-extendido.html) | 1 |
| <img src="https://raw.githubusercontent.com/JanGarciaC/Anti-Bloons3000/main/Hardware/Components/Imatges/ruedaloca.jpg" width="75"> | [Rueda loca mecànica](https://tienda.bricogeek.com/robotica/995-rueda-loca-plastico-abs-34.html) | 1 |
| <img src="https://tienda.bricogeek.com/933-large_default/pareja-de-ruedas-80x10mm-blanco.jpg" width="75"> | [Pareja de ruedas 80x10mm](https://tienda.bricogeek.com/robotica/303-pareja-de-ruedas-80x10mm-blanco.html) | 1 |

### Esquema del cablejat
L'esquema del cablejat que es mostra a continuació s'ha creat utilitzant el Cirkit Designer IDE
<div align="center"><img src="https://raw.githubusercontent.com/JanGarciaC/Anti-Bloons3000/main/Hardware/Esquemes/circuit.png" width="800"> </div> <br>

## Arquitectura del Software
<div align="center"><img src="https://raw.githubusercontent.com/JanGarciaC/Anti-Bloons3000/main/Software/arquitecturasoftware.jpeg" width="800"> </div> <br>

