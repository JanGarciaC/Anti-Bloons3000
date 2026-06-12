<h1 align="center">
  <br>
    <img src="https://raw.githubusercontent.com/JanGarciaC/Anti-Bloons3000/main/Galeria/bloonstd.png" alt="Tanc BloonsTD" width="250">
  <br>
    Anti-Bloons 3000
  <br>
</h1>

L'Anti-Bloons 3000 és un robot que detecta, restreja i elimina globus a distància. Inspirant-nos en la saga de jocs "Bloons TD", volem materialitzar un autòmat capaç de disparar projectils calculant la posició dels objectius (en el nostre cas, globus d'aire).

![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54) ![Raspberry Pi](https://img.shields.io/badge/-Raspberry_Pi-C51A4A?style=for-the-badge&logo=Raspberry-Pi) ![Fusion 360](https://img.shields.io/badge/Fusion_360-FF6600?style=for-the-badge&logo=autodesk&logoColor=white) ![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white) ![NumPy](https://img.shields.io/badge/NumPy-013243?style=for-the-badge&logo=numpy&logoColor=white) ![PyTorch](https://img.shields.io/badge/PyTorch-EE4C2C?style=for-the-badge&logo=pytorch&logoColor=white)

## Autors

- [Jan Garcia Comas](https://www.github.com/JanGarciaC) - Software Lead
- [Aarón Móstiga Móstiga](https://www.github.com/NNIU1708231) - 3D Parts & Mechanical Lead
- [Gerard Casanovas Urpí](https://www.github.com/NIU1708307) - Hardware Lead
- [Lluc Aymerich Medina](https://www.github.com/Insaly) - Testing & Validation Lead

# Com configurar-lo?

### Pas 1: Descarrega el repositori
Obra una terminal i executa la següent comanda:
```bash
git clone https://github.com/JanGarciaC/Anti-Bloons3000.git
cd Anti-Bloons3000
```

### Pas 2: Insta·la Python
Si no tens Python insta·lat, descarrega-te'l des de la [web oficial](https://www.python.org/downloads/)
Per a assegurar-nos que el software funcioni bé, utilitzarem la versió 3.13. Pots comprovar la teva versió de Python executant:
```bash
python --version
```

### Pas 3: Instal·la les dependències
Ara que ja tens Python descarregat, pots instal·lar les dependències necessàries simplement executant el fitxer setup.sh
```bash
bash setup.sh
```



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
| <img src="https://raw.githubusercontent.com/JanGarciaC/Anti-Bloons3000/main/Hardware/Components/Imatges/raspycamv2-8mp.jpg" width="75"> | [Càmera Raspberry Pi v2 - 8 Megapixels](https://tienda.bricogeek.com/sensores-imagen/822-camara-raspberry-pi-v2-8-megapixels.html) | 1 |
| <img src="https://raw.githubusercontent.com/JanGarciaC/Anti-Bloons3000/main/Hardware/Components/Imatges/motor.jpg" width="75"> | [Motor micro metall 30:1](https://tienda.bricogeek.com/motores-dc/1007-motor-micro-metal-30-1-hp-con-eje-extendido.html) | 1 |
| <img src="https://raw.githubusercontent.com/JanGarciaC/Anti-Bloons3000/main/Hardware/Components/Imatges/motor.jpg" width="75"> | [Motor micro metall 50:1](https://tienda.bricogeek.com/motores/115-motor-micro-metal-dc-con-reductora-50-1.html) | 2 |
| <img src="https://raw.githubusercontent.com/JanGarciaC/Anti-Bloons3000/main/Hardware/Components/Imatges/controlador.jpg" width="75"> | [Controlador Pololu A4988](https://tienda.bricogeek.com/controladores-motores/553-pololu-a4988-stepstick-prusa-reprap.html) | 2 |
| <img src="https://raw.githubusercontent.com/JanGarciaC/Anti-Bloons3000/main/Hardware/Components/Imatges/ruedaloca.jpg" width="75"> | [Roda boja mecànica](https://tienda.bricogeek.com/robotica/995-rueda-loca-plastico-abs-34.html) | 1 |
| <img src="https://raw.githubusercontent.com/JanGarciaC/Anti-Bloons3000/main/Hardware/Components/Imatges/rodesgrans.jpg" width="75"> | [Parella de rodes 80x10mm](https://tienda.bricogeek.com/robotica/303-pareja-de-ruedas-80x10mm-blanco.html) | 1 |
| <img src="https://raw.githubusercontent.com/JanGarciaC/Anti-Bloons3000/main/Hardware/Components/Imatges/rodespetites.jpg" width="75"> | [Parella de rodes 32x7mm](https://tienda.bricogeek.com/robotica/110-rueda-de-goma-32x7mm.html) | 1 |
| <img src="https://raw.githubusercontent.com/JanGarciaC/Anti-Bloons3000/main/Hardware/Components/Imatges/portapiles.jpg" width="75"> | [Portapiles x6 AA](https://www.leroymerlin.es/productos/portapilas-para-6-pilas-tipo-aa-86417903.html) | 1 |
| <img src="https://raw.githubusercontent.com/JanGarciaC/Anti-Bloons3000/main/Hardware/Components/Imatges/bateria.jpg" width="75"> | [Bateria portàtil 10000mah](https://www.cargadoresportatiles.es/version-mejorada-poweradd-pilot-2gs-power-bank-10000mah/) | 1 |
| <img src="https://raw.githubusercontent.com/JanGarciaC/Anti-Bloons3000/main/Hardware/Components/Imatges/servo.jpg" width="75"> | [Servo FEETECH FS5106B](https://tienda.bricogeek.com/motores/251-servo-feetech-fs5106b-6-kgcm.html) | 2 |
| <img src="https://tienda.bricogeek.com/8295-large_default/controlador-pwm-16-canales-i2c-pca9685.jpg" width="75"> | [Controlador de servos PCA9685] (https://tienda.bricogeek.com/controladores-motores/1764-controlador-pwm-16-canales-i2c-pca9685.html) | 1 |

### Esquema del cablejat
L'esquema del cablejat que es mostra a continuació s'ha creat utilitzant el Cirkit Designer IDE
<div align="center"><img src="https://raw.githubusercontent.com/JanGarciaC/Anti-Bloons3000/main/Hardware/Esquemes/circuit.png" width="800"> </div> <br>

## Arquitectura del Software
<div align="center"><img src="https://raw.githubusercontent.com/JanGarciaC/Anti-Bloons3000/main/Software/arquitecturasoftware.jpeg" width="800"> </div> <br>

## Software 

El software de l'Anti-Bloons conta de dues parts principals, una de visió per computador per a detectar globus i una altra que gestiona els pins de l'ordinador per a enfocar el canó cap a ells.

## Detecció de globus

El projecte entrena i executa models YOLO (un sistema basat en xarxes neuronals convolutives) per a detectar objectes. El repositori estès és [aquest](https://github.com/JanGarciaC/PractiquesVC/tree/main/Projecte)

### 1. Detecció
Per cada frame, YOLO passa la imatge per la xarxa neuronal entrenada i retorna bounding boxes (coordenades x1,y1,x2,y2) dels globus detectats, filtrant per un llindar de confiança i eliminant deteccions solapades amb NMS.
### 2. Rastreig
Com que YOLO detecta de nou cada frame sense memòria, el rastrejador assigna IDs persistents als globus entre frames. Ho fa calculant el centroide de cada bounding box i usant distància euclidiana per associar cada detecció nova amb l'objecte més proper del frame anterior.
### 3. Visualització
Per cada globus rastrejat, extreu el color dominant del seu interior (en espai HSV, filtrant ombres i blancs) i pinta la bounding box i la ID amb aquell mateix color saturat. A sobre hi afegeix un HUD amb el comptador de globus i els FPS.

<p align="center">
  <img src="https://raw.githubusercontent.com/JanGarciaC/Anti-Bloons3000/main/Galeria/globus.gif" alt="Exemple de detecció de globus" width="600">  
</p>

## Càlcul de l'angle del canó 
