# drive_pattern.py
# Beispiel: Zickzack-Muster für 1x1m Fläche
# Motorfunktionen müssen an deine Hardware angepasst werden!
import time

# Dummyfunktionen für Motorsteuerung (hier musst du GPIO/Motorcode einbauen)
def move_forward(cm, speed=0.5):
    print(f"Fahre {cm}cm vorwärts mit speed {speed}")
    # TODO: Motorsteuerung für vorwärts
    time.sleep(cm * 0.02)  # Dummy: Zeit proportional zur Strecke

def turn_left():
    print("Drehe 90° nach links")
    # TODO: Motorsteuerung für Drehung
    time.sleep(0.7)

def turn_right():
    print("Drehe 90° nach rechts")
    # TODO: Motorsteuerung für Drehung
    time.sleep(0.7)

def stop():
    print("Stoppe Motoren")
    # TODO: Motoren stoppen
    time.sleep(0.1)

# Zickzack-Muster für 1x1m Fläche (Bahnabstand: 20cm)
def clean_area_zigzag(area_size_cm=100, lane_width_cm=20):
    lanes = area_size_cm // lane_width_cm
    for lane in range(int(lanes)):
        move_forward(area_size_cm)
        if lane < lanes - 1:
            if lane % 2 == 0:
                turn_right()
                move_forward(lane_width_cm)
                turn_right()
            else:
                turn_left()
                move_forward(lane_width_cm)
                turn_left()
    # Letzte Bahn zurückfahren, falls nötig
    if lanes % 2 == 1:
        turn_right()
        turn_right()
        move_forward(area_size_cm)
    stop()

if __name__ == "__main__":
    print("Starte Zickzack-Reinigung für 1x1m Fläche...")
    clean_area_zigzag()
    print("Fertig!")
