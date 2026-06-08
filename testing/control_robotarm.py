import sys
import json
import time
import os
import platform
# import numpy as np
from pathlib import Path

# Detect the OS
system_name = platform.system().lower()

# Choose the correct path based on OS
if system_name == "windows":
    sdk_path = Path("fairino-python-sdk") / "windows" / "fairino"
elif system_name == "linux":
    sdk_path = Path("fairino-python-sdk") / "linux" / "fairino"
else:
    raise OSError(f"Unsupported operating system: {system_name}")

# Add the SDK path to sys.path
sys.path.append(str(sdk_path.resolve()))

# Import the RPC module
from Robot import RPC

# ---------------------------------------------------------
# konfig
# ---------------------------------------------------------
# fairino5
ROBOT_IP = "192.168.1.2"   
SPEED = 30  
TOOL = 0
USER = 0

rbt = RPC(ROBOT_IP)

# jodell
company=6
device=0
jawnumber = 1
pos = 0         # percentage range [0~100]
force = 50       # percentage [0~100]
vel = 30         # percentage [0 to 100]
maxtime = 10000  # waittime in ms [0~30000]
block = 0        # 0-blocking, 1-non-blocking
typ = 0
rotNum = 0
rotVel = 0
rotTorque = 0

# ---------------------------------------------------------
# initial
# ---------------------------------------------------------
def init_robot():
    """Stellt die Verbindung zum Roboter her und aktiviert ihn."""
    time.sleep(0.5)
    try:
        print(f"Verbunden mit Roboter {ROBOT_IP}")
        rbt.RobotEnable(1)
        rbt.Mode(1)  # 0 = Jog, 1 = Auto, 2 = Programm
        print("Roboter ist bereit.")
    except Exception as e:
        print("Fehler bei der Initialisierung:", e)
        rbt.RobotEnable(0)
        sys.exit()

def init_gripper(openingWidth: int=70):
    """Verbindet mit dem Greifer und aktiviert ihn."""
    jawnumber = 1
    ret = rbt.SetGripperConfig(company, device, softversion=0, bus=0)
    print("Configure gripper ", ret)
    time.sleep(1)
    error = rbt.ActGripper(jawnumber, 1)
    print("Activate gripper ", error)
    time.sleep(2)
    errorGripper = rbt.MoveGripper(jawnumber, openingWidth, 30, 30, 10000, 0, 0, 0, 0, 0)


# ---------------------------------------------------------
# looking for teached points as json
# ---------------------------------------------------------
def load_teach_points(teach_points_file):
    """Lädt die gespeicherten Teach Points aus einer JSON-Datei."""
    if not os.path.exists(teach_points_file):
        print(f"Fehler: Datei '{teach_points_file}' wurde nicht gefunden.")
        sys.exit()

    try:
        with open(teach_points_file, "r") as f:
            points = json.load(f)
        print(f"{len(points)} Teach Points erfolgreich geladen.")
        return points
    except Exception as e:
        print("Fehler beim Laden der Teach Points:", e)
        sys.exit()


# ---------------------------------------------------------
# driving
# ---------------------------------------------------------

def open_gripper(openingWidth: int=50):
    jawnumber = 1
    print("\n--- Start der Greifer-Bewegung ---")
    error1 = rbt.MoveGripper(jawnumber, openingWidth, 30, 30, 10000, 0, 0, 0, 0, 0)
    print("Greifer-Befehl Rückgabecode:", error1)

def close_gripper(openingWidth: int=90):
    jawnumber = 1
    print("\n--- Start der Greifer-Bewegung ---")
    error1 = rbt.MoveGripper(jawnumber, openingWidth, 50, 30, 10000, 0, 0, 0, 0, 0)
    print("Greifer-Befehl Rückgabecode:", error1)

def move_to_point(point_name, points):
    try:
        if point_name not in points:
            print(f"Punkt '{point_name}' wurde nicht gefunden.")
            return False
            
        coords = [float(x) for x in points[point_name][6:12]]
        
        print(f"Fahre zu Punkt '{point_name}': {coords}")
        ret = rbt.MoveJ(coords, TOOL, USER, vel=SPEED)
        print(f"Punkt '{point_name}' erreicht. Rückgabewert: {ret}")
        return True
    except Exception as e:
        print(f"Fehler beim Fahren zu Punkt '{point_name}':", e)
        return False

def move_to_rest_position(speed:int=30, acceleration: int=30):
    errorResting=0
    errorMove=0
    restAxisAngles = [
        -39,      #j1
        -63,      #j2
        -139,     #j3
        -158,     #j4
        -90,      #j5
        135,      #j6
    ]
    try:
        jointpos=get_joint_pos_degree()
        if(jointpos==None):
            errorResting=1
            return errorResting
        
        else:
            restAxisAngles[0]=jointpos[0]
            errorMove=rbt.MoveJ(restAxisAngles, tool=1, user=1, vel=speed, acc=acceleration)  
            if(errorMove==0):
                print("arrived safetyposition")
            else:
                print("can't move: {errorMove}")
                return errorMove
    except Exception as e:
        print(f"Ein unerwarteter Fehler bei der Bewegung ist aufgetreten: {e}")

def move_to_position(teach_points_file, speed: int = 30, acceleration: int = 30):
    # Vorhandene Funktion zum Laden nutzen
    points = load_teach_points(teach_points_file)

    # Ruheposition vorbereiten
    restAxisAngles = [
        None,     #j1
        -63,      #j2
        -139,     #j3
        -158,     #j4
        -90,      #j5
        135,      #j6
    ]
    
    jointpos = get_joint_pos_degree()

    if jointpos is None:
        print("Fehler: aktuelle Roboterposition konnte nicht gelesen werden.")
        return False
    else:
        restAxisAngles[0] = jointpos[0]

    try:
        for idx, (point_name, values) in enumerate(points.items()):
            coords = [float(x) for x in values[6:12]]

            if idx == 0 and point_name.lower() == "start":
                restAxisAngles[0] = coords[0]
                errorDrive = rbt.MoveJ(restAxisAngles, tool=1, user=1, vel=speed, acc=acceleration)
            else:
                errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=speed, acc=acceleration)

            print(f"Punkt '{point_name}' erreicht. Rückgabewert: {errorDrive}")

        return True

    except Exception as e:
        print(f"Fehler beim Fahren: {e}")
        return False


def test_line_movement(speed: int = 10, acceleration: int = 10):
 
     
     points = load_teach_points("test_line_movement.json")


     try:
         for idx, (point_name, values) in enumerate(points.items()):
             coords = [float(x) for x in values[0:6]]

             print(f"--- Koordinaten für Punkt '{point_name}' ---")
             print(f"X: {coords[0]}, Y: {coords[1]}, Z: {coords[2]}")
             print(f"Rx: {coords[3]}, Ry: {coords[4]}, Rz: {coords[5]}")
             print("---------------------------------------")

             if idx==1:
                 open_gripper(openingWidth=70)
                 errorDrive = rbt.MoveL(coords, tool=1, user=1, vel=speed, acc=acceleration)
             elif idx==2:
                 close_gripper(openingWidth=85)
                 errorDrive = rbt.MoveL(coords, tool=1, user=1, vel=speed, acc=acceleration)
             else:
                 errorDrive = rbt.MoveL(coords, tool=1, user=1, vel=speed, acc=acceleration)

             print(f"Punkt '{point_name}' erreicht. Rückgabewert: {errorDrive}")

         return True

     except Exception as e:
         print(f"Fehler beim Fahren: {e}")
         return False

def pick_up_item_opentrons(speed: int = 10, acceleration: int = 10, dangerSpeed: int = 10):
    points = load_teach_points("pick_up_opentrons.json")
    

    try:
        for idx, (point_name, values) in enumerate(points.items()):
            coords = [float(x) for x in values[6:12]]
            coordsLine = [float(x) for x in values[0:6]]
            
            if idx==2:
                open_gripper()
                #errorDrive = rbt.MoveL(coordsLine, tool=1, user=1, vel=dangerSpeed, acc=acceleration)
                errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=speed, acc=acceleration)
            elif idx==4:
                close_gripper()
                #errorDrive = rbt.MoveL(coordsLine, tool=1, user=1, vel=dangerSpeed, acc=acceleration)
                errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=speed, acc=acceleration)
            else:    
                errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=speed, acc=acceleration)
            
            print(f"Punkt '{point_name}' . Rückgabewert: {errorDrive}")
            
        return True

    except Exception as e:
        print(f"Fehler beim Fahren: {e}")
        return False

def release_item_opentrons(speed: int = 10, acceleration: int = 10, dangerSpeed: int = 10):
    points = load_teach_points("release_opentrons.json")

    try:
        for idx, (point_name, values) in enumerate(points.items()):
            coords = [float(x) for x in values[6:12]]
            coordsLine = [float(x) for x in values[0:6]]
            
            if idx==6:
                #errorDrive = rbt.MoveL(coordsLine, tool=1, user=1, vel=dangerSpeed, acc=acceleration)
                errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=speed, acc=acceleration)
                open_gripper()
            # elif idx==3:
            #     #errorDrive = rbt.MoveL(coordsLine, tool=1, user=1, vel=dangerSpeed, acc=acceleration)
            #     errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=speed, acc=acceleration)
            else:    
                errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=speed, acc=acceleration)
            
            print(f"Punkt '{point_name}' . Rückgabewert: {errorDrive}")
            
        return True

    except Exception as e:
        print(f"Fehler beim Fahren: {e}")
        return False

def get_joint_pos_degree():
    try:
        error, joint_position_degree = rbt.GetActualJointPosDegree()
        if error == 0:
            return joint_position_degree
        else:
            # Der RPC-Befehl ist angekommen, aber der Roboter meldet einen Fehler
            print(f"Fehler beim Abrufen der Gelenkinformationen: {error}")
            return None
    except Exception as e:
        # Der RPC-Befehl konnte nicht ausgeführt werden (z.B. Verbindung verloren)
        print(f"Ein unerwarteter Fehler ist aufgetreten: {e}")
        return None

def shutdown_robot():
    """Trennt die Verbindung zum Roboter."""
    try:
        rbt.RobotEnable(0)
        print("Roboter disabled.")
    except:
        print("Fehler beim Trennen der Verbindung.")

def pick_up_pickupstation(speed: int = 10, acceleration: int = 10, dangerSpeed: int =10):
    points = load_teach_points("pick_up_pickupstation.json")


    try:
        for idx, (point_name, values) in enumerate(points.items()):
            coords = [float(x) for x in values[6:12]]
            coordsLine = [float(x) for x in values[0:6]]
            
            if idx<=2:
                errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=speed, acc=acceleration)
            elif idx==3:
                #errorDrive = rbt.MoveL(coordsLine, tool=1, user=1, vel=dangerSpeed, acc=acceleration)
                errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=speed, acc=acceleration)
                open_gripper()
            elif idx==4:
                close_gripper()
                #errorDrive = rbt.MoveL(coordsLine, tool=1, user=1, vel=dangerSpeed, acc=acceleration)
                errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=speed, acc=acceleration)
            elif idx==5:
                #errorDrive = rbt.MoveL(coordsLine, tool=1, user=1, vel=speed, acc=acceleration)
                errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=speed, acc=acceleration)
            else:    
                errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=speed, acc=acceleration)
            
            print(f"Punkt '{point_name}' Rückgabewert: {errorDrive}")
            
        return True

    except Exception as e:
        print(f"Fehler beim Fahren: {e}")
        return False

def release_item_microscope(speed: int = 10, acceleration: int = 10, dangerSpeed: int =10):
    points = load_teach_points("release_microscope.json")

    try:
        for idx, (point_name, values) in enumerate(points.items()):
            coords = [float(x) for x in values[6:12]]
            coordsLine = [float(x) for x in values[0:6]]
            
            if idx==2:
                errorDrive = rbt.MoveL(coordsLine, tool=1, user=1, vel=speed, acc=acceleration)
            elif idx==4:
                errorDrive = rbt.MoveL(coordsLine, tool=1, user=1, vel=speed, acc=acceleration)
                open_gripper()
            elif idx==4:
                 errorDrive = rbt.MoveL(coordsLine, tool=1, user=1, vel=speed, acc=acceleration)
            elif idx==5:
                 errorDrive = rbt.MoveL(coordsLine, tool=1, user=1, vel=speed, acc=acceleration)
            else:    
                errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=speed, acc=acceleration)
            
            print(f"Punkt '{point_name}' erreicht. Rückgabewert: {errorDrive}")
            
        return True
    
    except Exception as e:
        print(f"Fehler beim Fahren: {e}")
        return False

def pick_up_microscope(speed: int = 10, acceleration: int = 10, dangerSpeed: int =10):
    points = load_teach_points("pick_microscope.json")

    try:
        for idx, (point_name, values) in enumerate(points.items()):
            coords = [float(x) for x in values[6:12]]
            coordsLine = [float(x) for x in values[0:6]]
            
            if idx==3:
                open_gripper()
                errorDrive = rbt.MoveL(coordsLine, tool=1, user=1, vel=dangerSpeed, acc=acceleration)
            elif idx==4:
                close_gripper()
                errorDrive = rbt.MoveL(coordsLine, tool=1, user=1, vel=dangerSpeed, acc=acceleration)
            elif idx==5:
                errorDrive = rbt.MoveL(coordsLine, tool=1, user=1, vel=dangerSpeed, acc=acceleration)
            else:    
                errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=speed, acc=acceleration)
                
            
            
            print(f"Punkt '{point_name}' erreicht. Rückgabewert: {errorDrive}")
            
        return True

    except Exception as e:
        print(f"Fehler beim Fahren: {e}")
        return False

def release_item_pickupstation(speed: int = 10, acceleration: int = 10, dangerSpeed: int =10):
    points = load_teach_points("release_pickupstation.json")


    try:
        for idx, (point_name, values) in enumerate(points.items()):
            coords = [float(x) for x in values[6:12]]
            coordsLine = [float(x) for x in values[0:6]]
            
            if idx<=1:
                errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=speed, acc=acceleration)
            elif idx==4:
                open_gripper()
                #errorDrive = rbt.MoveL(coordsLine, tool=1, user=1, vel=speed, acc=acceleration)
                errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=speed, acc=acceleration)
            elif idx==2:
                #errorDrive = rbt.MoveL(coordsLine, tool=1, user=1, vel=dangerSpeed, acc=acceleration)
                errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=speed, acc=acceleration)
            elif idx==3:
                #errorDrive = rbt.MoveL(coordsLine, tool=1, user=1, vel=dangerSpeed, acc=acceleration)
                errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=speed, acc=acceleration)
            else:    
                errorDrive = rbt.MoveJ(coords, tool=1, user=1, vel=speed, acc=acceleration)
            
            print(f"Punkt '{point_name}'. Rückgabewert: {errorDrive}")
            
        return True

    except Exception as e:
        print(f"Fehler beim Fahren: {e}")
        return False

# ---------------------------------------------------------
# Hauptprogramm
# ---------------------------------------------------------
if __name__ == "__main__":
    # Roboter initialisieren
    init_robot()
    init_gripper()
#%%  
    pick_up_pickupstation(40, 70, 10)
    # release_item_pickupstation(50, 70, 10)
    # pick_up_item_opentrons(10, 70, 10)
    # release_item_opentrons(10, 70, 10)
    # release_item_microscope(100, 70, 5)
    # sleep(1000)
    # pick_up_microscope(100, 70, 5)
    
    
    
#%%
    # Verbindung trennen
    #shutdown_robot()
