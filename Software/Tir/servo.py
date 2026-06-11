from gpiozero import AngularServo
from time import sleep

# Inicialitzem el servo al pin 12 (GPIO 12).
# Marquem el rang de 0 a 180 graus perquè el centre exacte siguin els 90.
# Els valors min_pulse_width i max_pulse_width (0.5ms a 2.5ms) són els estàndards 
# per a la majoria de servos petits (com els SG90 o MG996R) per fer els 180 graus complets.
servo = AngularServo(12, min_angle=0, max_angle=180, min_pulse_width=0.0005, max_pulse_width=0.0025)

def provar_servo():
    try:
        print("Posicionant el servo al centre (90 graus)...")
        servo.angle = 90
        sleep(2)  # Pausa perquè tinguis temps de veure on és el centre

        print("--> Iniciant moviment LENT (60 a 120 graus)")
        # Anem de 60 a 120 graus pas a pas
        for angle in range(60, 121, 1):
            servo.angle = angle
            sleep(0.05)  # Un sleep gran fa que el moviment es vegi lent i suau
            
        # Tornem de 120 a 60 graus
        for angle in range(120, 59, -1):
            servo.angle = angle
            sleep(0.05)
            
        sleep(1)

        print("--> Iniciant moviment RÀPID (60 a 120 graus)")
        # Fem salts més grans (de 5 en 5 graus) i reduïm el temps de sleep
        for angle in range(60, 121, 5):
            servo.angle = angle
            sleep(0.01)
            
        for angle in range(120, 59, -5):
            servo.angle = angle
            sleep(0.01)

        sleep(1)

        print("--> Iniciant moviments BRUSCOS (de cop)")
        servo.angle = 60
        sleep(0.5)
        servo.angle = 120
        sleep(0.5)
        
        print("Tornant al centre i acabant...")
        servo.angle = 90
        sleep(1)

    except KeyboardInterrupt:
        print("\nAturant la prova manualment...")
    finally:
        # És important alliberar el pin quan acabem per evitar vibracions al servo
        servo.detach()
        print("Test finalitzat.")

if __name__ == "__main__":
    provar_servo()