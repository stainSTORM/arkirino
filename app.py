# -*- coding: utf-8 -*-

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple
import os
import sys
import json
import time
import threading
import platform
from pathlib import Path
from arkitekt_next import easy, register
from rekuest_next.actors.sync import SyncGroup
from dotenv import load_dotenv


global_sync = SyncGroup("robot_sync")

# ------------------------------------------------------------------------------
# Fairino SDK import (Windows/Linux) with optional FAIRINO_SDK_DIR override
# ------------------------------------------------------------------------------
def _resolve_sdk_path() -> Optional[Path]:
    # Allow explicit override for CI/containers: FAIRINO_SDK_DIR=/path/to/fairino/.../fairino
    env_dir = os.getenv(
        "FAIRINO_SDK_DIR", "/home/pi/arkirino/fairino-python-sdk/fairino"
    )  # /home/pi/arkirino/linux/fairino
    if env_dir:
        return Path(env_dir)

    system_name = platform.system().lower()
    root = Path("fairino-python-sdk")
    if system_name == "windows":
        return (root / "windows" / "fairino").resolve()
    elif system_name == "linux":
        return (root / "linux" / "fairino").resolve()
    else:
        # Unsupported OS: caller will fall back to MockRPC
        return None


RPC = None  # will be assigned below

load_dotenv()

sdk_path = _resolve_sdk_path()
print(f"SDK path: {sdk_path}")
if sdk_path and sdk_path.exists():
    sys.path.append(str(sdk_path))
    try:
        from Robot import RPC as _RealRPC  # type: ignore

        RPC = _RealRPC
        print(f"Fairino SDK loaded from {sdk_path}")
    except Exception as e:
        print(f"Failed loading Fairino SDK from {sdk_path}: {e}")

if RPC is None:
    # Fallback to MockRPC for environments without the hardware SDK installed.
    try:
        from mock.MockRPC import MockRPC  # type: ignore
    except Exception:
        try:
            from MockRPC import MockRPC  # type: ignore
        except Exception:
            MockRPC = None  # No mock available

    if MockRPC is None:
        raise ImportError(
            "Neither Fairino SDK nor MockRPC is available. "
            "Install the SDK or provide a MockRPC for development."
        )
    RPC = MockRPC  # type: ignore
    print("Fairino SDK unavailable; using MockRPC fallback")


# ---------------------------------------------------------
# configuration
# ---------------------------------------------------------
# fairino5
ROBOT_IP = "192.168.50.200"
SPEED = 30
TOOL = 0
USER = 0


# gripper settings
COMPANY = 6
DEVICE = 0
JAWNUMBER = 1
POS = 0  # percentage range [0~100]
FORCE = 50  # percentage [0~100]
VELOCITY = 30  # percentage [0 to 100]
MAXTIME = 10000  # waittime in ms [0~30000]
IS_BLOCKING = 0  # 0-blocking, 1-non-blocking
TYP = 0
ROT_NUM = 0
ROT_VEL = 0
ROT_TORQUE = 0


rbt = RPC(ROBOT_IP)

IS_INITIALIZED = False

# ---------------------------------------------------------
# initialization
# ---------------------------------------------------------
def init() -> bool:
    global IS_INITIALIZED
    if IS_INITIALIZED:
        return True
    print("Start initializing robot and gripper")
    if not init_robot():
        return False
    if not init_gripper():
        return False
    print("Robot and gripper initialized")
    IS_INITIALIZED = True
    return True


def init_robot():
    """Establishes the connection to the robot and activates it."""
    time.sleep(0.5)
    try:
        print(f"Connected to robot {ROBOT_IP}")
        print(f"Robot object: {rbt}")
        rbt.RobotEnable(1)
        rbt.Mode(1)  # 0 = Jog, 1 = Auto, 2 = Program
        print("Robot is ready.")
    except Exception as e:
        print("Initialization error:", e)
        rbt.RobotEnable(0)
        return False
    return True


def init_gripper(openingWidth: int = 95):
    """Connects to the gripper and activates it."""
    try:
        jawnumber = 1
        ret = rbt.SetGripperConfig(COMPANY, DEVICE, softversion=0, bus=0)
        print("Configure gripper ", ret)
        time.sleep(1)
        error = rbt.ActGripper(jawnumber, 1)
        print("Activate gripper ", error)
        time.sleep(2)
        errorGripper = rbt.MoveGripper(
            jawnumber, openingWidth, 30, 30, 10000, 0, 0, 0, 0, 0
        )
    except Exception as e:
        print("Error initializing gripper:", e)
        return False
    return True


# ---------------------------------------------------------
# loading taught points as JSON
# ---------------------------------------------------------
def load_teach_points(teach_points_file) -> Optional[Dict[str, List[float]]]:
    """Loads the stored teach points from a JSON file."""
    if not os.path.exists(teach_points_file):
        print(f"Error: File '{teach_points_file}' not found.")
        return None

    try:
        with open(teach_points_file, "r") as f:
            points = json.load(f)
        print(f"{len(points)} teach points loaded successfully.")
        return points
    except Exception as e:
        print("Error loading teach points:", e)
        return None


# ---------------------------------------------------------
# driving
# ---------------------------------------------------------


def open_gripper(openingWidth: int = 50):
    jawnumber = 1
    print("\n--- Starting gripper movement ---")
    error1 = rbt.MoveGripper(jawnumber, openingWidth, 30, 30, 10000, 0, 0, 0, 0, 0)
    print("Gripper command return code:", error1)


def close_gripper(openingWidth: int = 95):
    jawnumber = 1
    print("\n--- Starting gripper movement ---")
    error1 = rbt.MoveGripper(jawnumber, openingWidth, 50, 30, 10000, 0, 0, 0, 0, 0)
    print("Gripper command return code:", error1)


def move_to_point(point_name, points):
    try:
        if point_name not in points:
            print(f"Point '{point_name}' not found.")
            return False

        coords = [float(x) for x in points[point_name][6:12]]

        print(f"Moving to point '{point_name}': {coords}")
        ret = rbt.MoveJ(coords, TOOL, USER, vel=SPEED)
        print(f"Point '{point_name}' reached. Return value: {ret}")
        return True
    except Exception as e:
        print(f"Error moving to point '{point_name}':", e)
        return False


def move_to_position(teach_points_file, speed: int = 30, acceleration: int = 30):
    # Vorhandene Funktion zum Laden nutzen
    points = load_teach_points(teach_points_file)
    if points is None:
        return False

    # Ruheposition vorbereiten
    restAxisAngles = [
        None,  # j1
        -63,  # j2
        -139,  # j3
        -158,  # j4
        -90,  # j5
        135,  # j6
    ]

    jointpos = get_joint_pos_degree()

    if jointpos is None:
        print("Error: Current robot position could not be read.")
        return False
    else:
        restAxisAngles[0] = jointpos[0]

    try:
        for idx, (point_name, values) in enumerate(points.items()):
            coords = [float(x) for x in values[6:12]]

            if idx == 0 and point_name.lower() == "start":
                restAxisAngles[0] = coords[0]
                errorDrive = rbt.MoveJ(
                    restAxisAngles, tool=1, user=1, vel=speed, acc=acceleration
                )
            else:
                errorDrive = rbt.MoveJ(
                    coords, tool=1, user=1, vel=speed, acc=acceleration
                )

            print(f"Point '{point_name}' reached. Return value: {errorDrive}")

        return True

    except Exception as e:
        print(f"Error while moving: {e}")
        return False


def pick_up_item_opentrons(
    speed: int = 10, acceleration: int = 10, dangerSpeed: int = 10
):
    points = load_teach_points("./control_points/pick_opentrons.json")

    try:
        for idx, (point_name, values) in enumerate(points.items()):
            coords = [float(x) for x in values[6:12]]
            coordsLine = [float(x) for x in values[0:6]]

            if idx == 2:
                open_gripper()
                errorDrive = rbt.MoveL(
                    coordsLine, tool=1, user=1, vel=dangerSpeed, acc=acceleration
                )
            elif idx == 3:
                close_gripper()
                errorDrive = rbt.MoveL(
                    coordsLine, tool=1, user=1, vel=dangerSpeed, acc=acceleration
                )
            else:
                errorDrive = rbt.MoveJ(
                    coords, tool=1, user=1, vel=speed, acc=acceleration
                )

            print(f"Point '{point_name}'. Return value: {errorDrive}")

        return True

    except Exception as e:
        print(f"Error while moving: {e}")
        return False


def release_item_opentrons(
    speed: int = 10, acceleration: int = 10, dangerSpeed: int = 10
):
    points = load_teach_points("./control_points/release_opentrons.json")

    try:
        for idx, (point_name, values) in enumerate(points.items()):
            coords = [float(x) for x in values[6:12]]
            coordsLine = [float(x) for x in values[0:6]]

            if idx == 2:
                errorDrive = rbt.MoveL(
                    coordsLine, tool=1, user=1, vel=dangerSpeed, acc=acceleration
                )
                open_gripper()
            elif idx == 3:
                errorDrive = rbt.MoveL(
                    coordsLine, tool=1, user=1, vel=dangerSpeed, acc=acceleration
                )
            else:
                errorDrive = rbt.MoveJ(
                    coords, tool=1, user=1, vel=speed, acc=acceleration
                )

            print(f"Point '{point_name}'. Return value: {errorDrive}")

        return True

    except Exception as e:
        print(f"Error while moving: {e}")
        return False


def get_joint_pos_degree():
    try:
        error, joint_position_degree = rbt.GetActualJointPosDegree()
        if error == 0:
            return joint_position_degree
        else:
            # The RPC command arrived, but the robot reported an error
            print(f"Error retrieving joint information: {error}")
            return None
    except Exception as e:
        # The RPC command could not be executed (e.g. connection lost)
        print(f"An unexpected error occurred: {e}")
        return None


def shutdown_robot():
    """Disconnects from the robot."""
    global IS_INITIALIZED
    try:
        rbt.RobotEnable(0)
        print("Robot shut down.")
        IS_INITIALIZED = False
    except:
        print("Error disconnecting.")


def pick_up_pickupstation(
    speed: int = 10, acceleration: int = 10, dangerSpeed: int = 10
):
    points = load_teach_points("./control_points/pick_up_pickupstation.json")

    try:
        for idx, (point_name, values) in enumerate(points.items()):
            coords = [float(x) for x in values[6:12]]
            coordsLine = [float(x) for x in values[0:6]]

            if idx <= 2:
                errorDrive = rbt.MoveJ(
                    coords, tool=1, user=1, vel=speed, acc=acceleration
                )
            elif idx == 3:
                open_gripper()
                errorDrive = rbt.MoveL(
                    coordsLine, tool=1, user=1, vel=dangerSpeed, acc=acceleration
                )
            elif idx == 4:
                close_gripper()
                errorDrive = rbt.MoveL(
                    coordsLine, tool=1, user=1, vel=dangerSpeed, acc=acceleration
                )
            elif idx == 5:
                errorDrive = rbt.MoveL(
                    coordsLine, tool=1, user=1, vel=speed, acc=acceleration
                )
            else:
                errorDrive = rbt.MoveJ(
                    coords, tool=1, user=1, vel=speed, acc=acceleration
                )

            print(f"Point '{point_name}'. Return value: {errorDrive}")

        return True

    except Exception as e:
        print(f"Error while moving: {e}")
        return False


def release_item_microscope(
    speed: int = 10, acceleration: int = 10, dangerSpeed: int = 10
):
    points = load_teach_points("./control_points/release_microscope.json")

    try:
        for idx, (point_name, values) in enumerate(points.items()):
            coords = [float(x) for x in values[6:12]]
            coordsLine = [float(x) for x in values[0:6]]

            if idx == 2:
                errorDrive = rbt.MoveL(
                    coordsLine, tool=1, user=1, vel=speed, acc=acceleration
                )
            elif idx == 3:
                errorDrive = rbt.MoveL(
                    coordsLine, tool=1, user=1, vel=speed, acc=acceleration
                )
                open_gripper()
            elif idx == 4:
                errorDrive = rbt.MoveL(
                    coordsLine, tool=1, user=1, vel=speed, acc=acceleration
                )
            elif idx == 5:
                errorDrive = rbt.MoveL(
                    coordsLine, tool=1, user=1, vel=speed, acc=acceleration
                )
            else:
                errorDrive = rbt.MoveJ(
                    coords, tool=1, user=1, vel=speed, acc=acceleration
                )

            print(f"Point '{point_name}'. Return value: {errorDrive}")

        return True

    except Exception as e:
        print(f"Error while moving: {e}")
        return False


def pick_up_microscope(speed: int = 10, acceleration: int = 10, dangerSpeed: int = 10):
    points = load_teach_points("./control_points/pick_microscope.json")

    try:
        for idx, (point_name, values) in enumerate(points.items()):
            coords = [float(x) for x in values[6:12]]
            coordsLine = [float(x) for x in values[0:6]]

            if idx == 3:
                open_gripper()
                errorDrive = rbt.MoveL(
                    coordsLine, tool=1, user=1, vel=dangerSpeed, acc=acceleration
                )
            elif idx == 4:
                close_gripper()
                errorDrive = rbt.MoveL(
                    coordsLine, tool=1, user=1, vel=dangerSpeed, acc=acceleration
                )
            elif idx == 5:
                errorDrive = rbt.MoveL(
                    coordsLine, tool=1, user=1, vel=dangerSpeed, acc=acceleration
                )
            else:
                errorDrive = rbt.MoveJ(
                    coords, tool=1, user=1, vel=speed, acc=acceleration
                )

            print(f"Point '{point_name}'. Return value: {errorDrive}")

        return True

    except Exception as e:
        print(f"Error while moving: {e}")
        return False


def release_item_pickupstation(
    speed: int = 10, acceleration: int = 10, dangerSpeed: int = 10
):
    points = load_teach_points("./control_points/release_pickupstation.json")

    try:
        for idx, (point_name, values) in enumerate(points.items()):
            coords = [float(x) for x in values[6:12]]
            coordsLine = [float(x) for x in values[0:6]]

            if idx <= 1:
                errorDrive = rbt.MoveJ(
                    coords, tool=1, user=1, vel=speed, acc=acceleration
                )
            elif idx == 4:
                open_gripper()
                errorDrive = rbt.MoveL(
                    coordsLine, tool=1, user=1, vel=speed, acc=acceleration
                )
            elif idx == 2:
                errorDrive = rbt.MoveL(
                    coordsLine, tool=1, user=1, vel=dangerSpeed, acc=acceleration
                )
            elif idx == 3:
                errorDrive = rbt.MoveL(
                    coordsLine, tool=1, user=1, vel=dangerSpeed, acc=acceleration
                )
            else:
                errorDrive = rbt.MoveJ(
                    coords, tool=1, user=1, vel=speed, acc=acceleration
                )

            print(f"Point '{point_name}'. Return value: {errorDrive}")

        return True

    except Exception as e:
        print(f"Error while moving: {e}")
        return False


@register(sync=global_sync)
def pickup_slide_from_pickupstation():
    if not init():
        print("Could not start routine. Exiting.")
        return
    print("Picking up slide from pickup station")
    pick_up_pickupstation(speed=40, acceleration=50, dangerSpeed=10)

@register(sync=global_sync)
def move_slide_to_opentron():
    if not init():
        print("Could not start routine. Exiting.")
        return
    print("Moving slide to opentron")
    release_item_opentrons(speed=40, acceleration=50, dangerSpeed=10)


@register(sync=global_sync)
def pickup_slide_from_opentron():
    if not init():
        print("Could not start routine. Exiting.")
        return
    print("Picking up slide from opentron")
    pick_up_item_opentrons(speed=40, acceleration=50, dangerSpeed=10)


@register(sync=global_sync)
def move_slide_to_microscope():
    if not init():
        print("Could not start routine. Exiting.")
        return
    print("Moving slide to microscope")
    release_item_microscope(speed=40, acceleration=50, dangerSpeed=10)

@register(sync=global_sync)
def pickup_slide_from_microscope():
    if not init():
        print("Could not start routine. Exiting.")
        return
    print("Picking up slide from microscope")
    pick_up_microscope(speed=40, acceleration=50, dangerSpeed=10)


@register(sync=global_sync)
def move_slide_to_pickupstation():
    if not init():
        print("Could not start routine. Exiting.")
        return
    print("Moving slide to pickup station")
    release_item_pickupstation(speed=40, acceleration=50, dangerSpeed=10)

@register(sync=global_sync)
def shutdown():
    shutdown_robot()

@register(sync=global_sync)
def complete_sequence_once():
    if not init():
        print("Could not start routine. Exiting.")
        return
    print("Completing sequence once")
    pickup_slide_from_pickupstation()
    move_slide_to_opentron()
    pickup_slide_from_opentron()
    move_slide_to_microscope()
    pickup_slide_from_microscope()
    move_slide_to_pickupstation()
    shutdown_robot()

if __name__ == "__main__":
    app_name = os.getenv("ARKITEKT_APPNAME", "arkirino")
    if app_name == "":
        print("ARKITEKT_APPNAME is not set. Please set the ARKITEKT_APPNAME environment variable. For example put it in .env file.")
        exit(1)
    app_url = os.getenv("ARKITEKT_URL", "go.arkitekt.live")
    app = easy(identifier=app_name, url=app_url, redeem_token=os.getenv("REDEEM_TOKEN"))
    app.enter()
    app.run()
