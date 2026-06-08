# =============================================================================
# take the points including location and meta data from the robot
# need the names from the points you want to get line 69
# =============================================================================
import sys
import json
import time
from pathlib import Path

pfad = Path("fairino-python-sdk") / "windows" / "fairino"
sys.path.append(str(pfad.resolve()))

from Robot import RPC

# ---------------------------------------------------------
# Konfiguration
# ---------------------------------------------------------
ROBOT_IP = "192.168.1.2"   # IP-Adresse des Roboters
OUTPUT_FILE = "teach_points.json"

# Roboterobjekt erstellen
rbt = RPC(ROBOT_IP)

def init_robot():
    """Initialisiert den Roboter und aktiviert ihn"""
    try:
        print(f"Verbunden mit Roboter {ROBOT_IP}")
        rbt.RobotEnable(1)
        rbt.Mode(1)  # 1 = Auto
        print("Roboter ist bereit.")
    except Exception as e:
        print("Fehler bei der Initialisierung:", e)
        rbt.RobotEnable(0)
        sys.exit()

def save_teach_points(file_path, point_names):
    """Liest die angegebenen Teach Points aus und speichert sie"""
    all_points = {}
    try:
        for name in point_names:
            error, data = rbt.GetRobotTeachingPoint(name)
            if error == 0 and data:
                all_points[name] = data
                print(f"Punkt '{name}' erfolgreich gelesen: {data}")
            else:
                print(f"Fehler beim Lesen von '{name}', ErrorCode={error}")

        # Speichern in JSON
        with open(file_path, "w") as f:
            json.dump(all_points, f, indent=4)

        print(f"\nTeach Points wurden erfolgreich in '{file_path}' gespeichert.")

    except Exception as e:
        print("Fehler beim Auslesen oder Speichern der Teach Points:", e)

def shutdown_robot():
    """Verbindung zum Roboter trennen"""
    try:
        rbt.RobotEnable(0)
        print("Roboter heruntergefahren.")
    except Exception as e:
        print("Fehler beim Trennen der Verbindung:", e)

# ---------------------------------------------------------
# Hauptprogramm
# ---------------------------------------------------------
if __name__ == "__main__":
    init_robot()

    # Liste der Punktnamen anpassen → musst du vorher im Teach-Interface sehen
    teach_point_names = ["home", "start", "move1", "move2", "move3", "move4", "move5", "move6", "move7", "move8", "move9", "move10", "release", "collect"]

    save_teach_points(OUTPUT_FILE, teach_point_names)
    time.sleep(2)
    shutdown_robot()
