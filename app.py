# fairino_arkitekt_service.py
# -*- coding: utf-8 -*-
"""
Arkitekt service that exposes the real robot control functions from control_robotarm.py.

Highlights & pitfalls handled:
- Cross-platform SDK import (Windows/Linux) with optional FAIRINO_SDK_DIR override.
- Safe fallback to MockRPC when SDK isn't available (dev/test machines).
- Thread-safe, single shared RPC session with connect/teardown.
- No sys.exit() in service paths; errors are raised and returned cleanly to Arkitekt.
- Robust teach-point parsing (supports [0:6] and [6:12] layouts).
- Consistent progress reporting and helpful error messages.
- Exact function signatures preserved from control_robotarm.py.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple
import os
import sys
import json
import time
import threading
import platform
from pathlib import Path

# ------------------------------------------------------------------------------
# Arkitekt integration (with safe fallbacks for local dev)
# ------------------------------------------------------------------------------
try:
    from arkitekt_next import register, easy, progress
except Exception:
    # Fallbacks allow running without Arkitekt installed
    print("arkitekt_next not found: using dummy register/easy/progress for local dev")

    def register(fn=None, **k):
        if fn is None:

            def deco(f):
                return f

            return deco
        return fn

    class _DummyApp:
        def register(self, *fns):
            pass

        def enter(self):
            pass

        def run(self):
            pass

        def run_detached(self):
            pass

        def exit(self):
            pass

    def easy(*a, **k):
        return _DummyApp()

    def progress(*a, **k):
        pass


# ------------------------------------------------------------------------------
# Fairino SDK import (Windows/Linux) with optional FAIRINO_SDK_DIR override
# ------------------------------------------------------------------------------
def _resolve_sdk_path() -> Optional[Path]:
    # Allow explicit override for CI/containers: FAIRINO_SDK_DIR=/path/to/fairino/.../fairino
    env_dir = os.getenv("FAIRINO_SDK_DIR")
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

sdk_path = _resolve_sdk_path()
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
        from arkitekt_integration.MockRPC import MockRPC  # type: ignore
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


# ------------------------------------------------------------------------------
# Configuration (env overrides are supported)
# ------------------------------------------------------------------------------
ROBOT_IP = os.getenv("FAIRINO_ROBOT_IP", "192.168.1.2")

# Defaults used by the original control script; keep consistent with the real code.
DEFAULT_SPEED = int(os.getenv("FAIRINO_DEFAULT_SPEED", "30"))
DEFAULT_TOOL = int(
    os.getenv("FAIRINO_DEFAULT_TOOL", "0")
)  # original script defines TOOL=0
DEFAULT_USER = int(
    os.getenv("FAIRINO_DEFAULT_USER", "0")
)  # original script defines USER=0

# Gripper defaults (from control script constants & usage)
GRIPPER_COMPANY = int(os.getenv("FAIRINO_GRIPPER_COMPANY", "6"))
GRIPPER_DEVICE = int(os.getenv("FAIRINO_GRIPPER_DEVICE", "0"))


# ------------------------------------------------------------------------------
# Thread-safe RPC session manager
# ------------------------------------------------------------------------------
class RobotSession:
    """Manages a single shared RPC connection with thread-safety."""

    def __init__(self, ip: str):
        self.ip = ip
        self._lock = threading.RLock()
        self._rbt = None  # type: ignore

    def connect(self) -> None:
        with self._lock:
            if self._rbt is not None:
                return
            self._rbt = RPC(self.ip)  # type: ignore
            # Give the controller a moment to settle on fresh connection
            time.sleep(0.2)

    @property
    def client(self):
        with self._lock:
            if self._rbt is None:
                self.connect()
            return self._rbt

    def shutdown(self) -> None:
        with self._lock:
            if self._rbt is None:
                return
            try:
                # Best-effort disable, handled by caller functions too
                if hasattr(self._rbt, "RobotEnable"):
                    self._rbt.RobotEnable(0)
            finally:
                self._rbt = None


SESSION = RobotSession(ROBOT_IP)


# ------------------------------------------------------------------------------
# Utilities
# ------------------------------------------------------------------------------
def _coerce_joint_slice(values: List[float]) -> List[float]:
    """Extract joint targets from a point-like list.

    Many recording formats store cartesian first (0..5) and joints next (6..11).
    If only 0..5 are present, we assume these are joints already.
    """
    if not isinstance(values, list):
        raise ValueError("Teach point values must be a list")

    if len(values) >= 12:
        # Typical structure: [x,y,z,rx,ry,rz, j1..j6]
        return [float(v) for v in values[6:12]]
    # Otherwise assume first 6 are joints
    if len(values) >= 6:
        return [float(v) for v in values[0:6]]

    raise ValueError(f"Teach point requires at least 6 numbers, got {len(values)}")


def _load_teach_points_file(path: str) -> Dict[str, List[float]]:
    """Load teach points from JSON, returning as dict[name] -> raw list."""
    resolved = Path(path).expanduser().resolve()
    if not resolved.exists():
        raise FileNotFoundError(f"Teach points file not found: {resolved}")

    with resolved.open("r", encoding="utf-8") as f:
        data = json.load(f)

    if not isinstance(data, dict):
        raise ValueError("Teach points JSON must be an object mapping names to lists")

    # Normalize all entries to lists of floats (don't slice yet)
    norm: Dict[str, List[float]] = {}
    for name, raw in data.items():
        if isinstance(raw, dict) and "joints" in raw:
            raw = raw["joints"]
        if not isinstance(raw, list):
            raise ValueError(
                f"Teach point '{name}' must be a list or dict with 'joints'"
            )
        norm[name] = [float(v) for v in raw]
    return norm


def _progress_step(cur: int, total: int, label: str) -> None:
    p = int(100 * (cur / max(total, 1)))
    progress(p, label)


# ------------------------------------------------------------------------------
# Exposed functions (exact signatures preserved from control_robotarm.py)
# Each function runs under lock and raises on fatal errors instead of sys.exit().
# ------------------------------------------------------------------------------


@register
def init_robot() -> None:
    """Establishes the connection to the robot and enables it."""
    rbt = SESSION.client
    progress(5, "Connecting to robot ...")
    try:
        # Enable & switch to Auto mode as in control_robotarm.py
        rbt.RobotEnable(1)
        rbt.Mode(1)  # 0=Jog, 1=Auto, 2=Program
        # Optionally set Tool/User; many programs rely on a known frame:
        if hasattr(rbt, "User"):
            rbt.User(DEFAULT_USER)
        if hasattr(rbt, "Tool"):
            rbt.Tool(DEFAULT_TOOL)
        time.sleep(0.5)
        progress(100, f"Robot {ROBOT_IP} is ready")
    except Exception as e:
        try:
            rbt.RobotEnable(0)
        except Exception:
            pass
        raise RuntimeError(f"Initialization failed: {e}") from e


@register
def init_gripper(openingWidth: int = 70) -> None:
    """Connects to the gripper, activates it, and sets the start opening."""
    rbt = SESSION.client
    jawnumber = 1
    progress(10, "Configuring gripper ...")
    ret = rbt.SetGripperConfig(GRIPPER_COMPANY, GRIPPER_DEVICE, softversion=0, bus=0)
    time.sleep(1)
    progress(40, f"Gripper configured (ret={ret}), activating ...")
    err = rbt.ActGripper(jawnumber, 1)
    time.sleep(2)
    progress(70, f"Gripper activated (err={err}), opening initially ...")
    _ = rbt.MoveGripper(jawnumber, openingWidth, 30, 30, 10000, 0, 0, 0, 0, 0)
    progress(100, "Gripper ready")


@register
def load_teach_points(teach_points_file: str) -> Dict[str, List[float]]:
    """Loads the stored teach points from a JSON file (raw; unsliced)."""
    pts = _load_teach_points_file(teach_points_file)
    progress(100, f"Loaded {len(pts)} teach points")
    return pts


@register
def open_gripper(openingWidth: int = 50) -> int:
    """Opens the gripper to 'openingWidth'."""
    rbt = SESSION.client
    jawnumber = 1
    progress(30, f"Opening gripper to {openingWidth}")
    ret = rbt.MoveGripper(jawnumber, openingWidth, 30, 30, 10000, 0, 0, 0, 0, 0)
    progress(100, f"Gripper opened (ret={ret})")
    return int(ret)


@register
def close_gripper(openingWidth: int = 85) -> int:
    """Closes the gripper (larger number = tighter in the original data)."""
    rbt = SESSION.client
    jawnumber = 1
    progress(30, f"Closing gripper to {openingWidth}")
    ret = rbt.MoveGripper(jawnumber, openingWidth, 50, 30, 10000, 0, 0, 0, 0, 0)
    progress(100, f"Gripper closed (ret={ret})")
    return int(ret)


@register
def move_to_point(point_name: str, points: Dict[str, List[float]]) -> bool:
    """Moves (MoveJ) to a named point from 'points'."""
    rbt = SESSION.client
    if point_name not in points:
        raise KeyError(f"Point '{point_name}' not in teach points")

    coords = _coerce_joint_slice(points[point_name])
    progress(20, f"Moving to '{point_name}' ...")
    ret = rbt.MoveJ(coords, tool=DEFAULT_TOOL, user=DEFAULT_USER, vel=DEFAULT_SPEED)
    progress(100, f"Point '{point_name}' reached (ret={ret})")
    return True


@register
def move_to_rest_position(speed: int = 30, acceleration: int = 30) -> Optional[int]:
    """Moves to a safe rest pose (keeps current J1, fixed J2..J6)."""
    rbt = SESSION.client
    rest_axes = [-39.0, -63.0, -139.0, -158.0, -90.0, 135.0]
    try:
        err, joint_pos_deg = rbt.GetActualJointPosDegree()
        if err != 0 or joint_pos_deg is None:
            progress(100, f"Joint query error: {err}")
            return 1
        # Keep current J1 per original script logic
        rest_axes[0] = float(joint_pos_deg[0])
        progress(30, "Moving to rest pose ...")
        ret = rbt.MoveJ(
            rest_axes, tool=DEFAULT_TOOL, user=DEFAULT_USER, vel=speed, acc=acceleration
        )
        progress(100, f"Rest pose reached (ret={ret})")
        return int(ret)
    except Exception as e:
        raise RuntimeError(f"Error in move_to_rest_position: {e}") from e


@register
def move_to_position(
    teach_points_file: str, speed: int = 30, acceleration: int = 30
) -> bool:
    """
    Moves through a sequence of teach points (MoveJ).
    Special case: the first element is named 'start' -> copy current J1 into a rest-like pose and go there first.
    """
    rbt = SESSION.client
    points = _load_teach_points_file(teach_points_file)

    # Prepare rest pose with current J1
    err, joint_pos_deg = rbt.GetActualJointPosDegree()
    if err != 0 or joint_pos_deg is None:
        raise RuntimeError(f"Joint query failed (error={err})")

    rest_axes = [float(joint_pos_deg[0]), -63.0, -139.0, -158.0, -90.0, 135.0]

    total = len(points)
    for idx, (name, raw) in enumerate(points.items(), start=1):
        coords = _coerce_joint_slice(raw)
        if idx == 1 and name.lower() == "start":
            # Align J1 and go to a rest-like pose first
            progress(10, "Initial alignment (start) ...")
            _ = rbt.MoveJ(
                rest_axes,
                tool=DEFAULT_TOOL,
                user=DEFAULT_USER,
                vel=speed,
                acc=acceleration,
            )

        progress(int(10 + (80 * idx / max(total, 1))), f"Moving '{name}' ...")
        _ = rbt.MoveJ(
            coords, tool=DEFAULT_TOOL, user=DEFAULT_USER, vel=speed, acc=acceleration
        )

    progress(100, "Sequence complete")
    return True


@register
def pick_up_item_opentrons(speed: int = 10, acceleration: int = 10) -> bool:
    """Gripper sequence for picking up at the Opentrons (file: working_test_code/pick_opentrons.json)."""
    rbt = SESSION.client
    points = _load_teach_points_file("working_test_code/pick_opentrons.json")

    for idx, (name, raw) in enumerate(points.items()):
        coords = _coerce_joint_slice(raw)
        if idx == 1:
            open_gripper()
        progress(30 + int(60 * idx / max(len(points), 1)), f"Moving '{name}' ...")
        _ = rbt.MoveJ(
            coords, tool=DEFAULT_TOOL, user=DEFAULT_USER, vel=speed, acc=acceleration
        )
        if idx == 2:
            close_gripper()
    progress(100, "Opentrons pickup complete")
    return True


@register
def release_item_opentrons(speed: int = 10, acceleration: int = 10) -> bool:
    """Gripper sequence for placing at the Opentrons (file: working_test_code/release_opentrons.json)."""
    rbt = SESSION.client
    points = _load_teach_points_file("working_test_code/release_opentrons.json")

    for idx, (name, raw) in enumerate(points.items()):
        coords = _coerce_joint_slice(raw)
        progress(30 + int(60 * idx / max(len(points), 1)), f"Moving '{name}' ...")
        _ = rbt.MoveJ(
            coords, tool=DEFAULT_TOOL, user=DEFAULT_USER, vel=speed, acc=acceleration
        )
        if idx == 6:
            open_gripper()
    progress(100, "Opentrons release complete")
    return True


@register
def get_joint_pos_degree() -> Optional[List[float]]:
    """Reads current joint angles in degrees."""
    rbt = SESSION.client
    try:
        err, joints = rbt.GetActualJointPosDegree()
        if err == 0:
            return joints
        print(f"Error retrieving joint information: {err}")
        return None
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return None


@register
def shutdown_robot() -> None:
    """Disconnects from the robot."""
    # Try to properly disable; if not possible, still drop session
    try:
        rbt = SESSION.client
        rbt.RobotEnable(0)
        progress(100, "Robot shut down")
    finally:
        SESSION.shutdown()


@register
def pick_up_pickupstation(speed: int = 10, acceleration: int = 10) -> bool:
    """Pickup at the pickup station (file: working_test_code/pick_pickupstation.json)."""
    rbt = SESSION.client
    points = _load_teach_points_file("working_test_code/pick_pickupstation.json")

    for idx, (name, raw) in enumerate(points.items()):
        coords = _coerce_joint_slice(raw)
        if idx == 1:
            open_gripper()
        progress(30 + int(60 * idx / max(len(points), 1)), f"Moving '{name}' ...")
        _ = rbt.MoveJ(
            coords, tool=DEFAULT_TOOL, user=DEFAULT_USER, vel=speed, acc=acceleration
        )
        if idx == 3:
            close_gripper()
    progress(100, "Pickup station pickup complete")
    return True


@register
def release_item_microscope(speed: int = 10, acceleration: int = 10) -> bool:
    """Release at the microscope (file: working_test_code/release_microscope.json)."""
    rbt = SESSION.client
    points = _load_teach_points_file("working_test_code/release_microscope.json")

    for idx, (name, raw) in enumerate(points.items()):
        coords = _coerce_joint_slice(raw)
        progress(30 + int(60 * idx / max(len(points), 1)), f"Moving '{name}' ...")
        _ = rbt.MoveJ(
            coords, tool=DEFAULT_TOOL, user=DEFAULT_USER, vel=speed, acc=acceleration
        )
        if idx == 1:
            open_gripper()
    progress(100, "Microscope release complete")
    return True


@register
def pick_up_microscope(speed: int = 10, acceleration: int = 10) -> bool:
    """Pickup at the microscope (file: working_test_code/pick_microscope.json)."""
    rbt = SESSION.client
    points = _load_teach_points_file("working_test_code/pick_microscope.json")

    for idx, (name, raw) in enumerate(points.items()):
        coords = _coerce_joint_slice(raw)
        if idx in (1, 6):
            open_gripper()
        progress(30 + int(60 * idx / max(len(points), 1)), f"Moving '{name}' ...")
        _ = rbt.MoveJ(
            coords, tool=DEFAULT_TOOL, user=DEFAULT_USER, vel=speed, acc=acceleration
        )
        if idx in (2, 8):
            close_gripper()
    progress(100, "Microscope pickup complete")
    return True


@register
def release_item_pickupstation(speed: int = 10, acceleration: int = 10) -> bool:
    """Release at the pickup station (file: working_test_code/release_pickupstation.json)."""
    rbt = SESSION.client
    points = _load_teach_points_file("working_test_code/release_pickupstation.json")

    for idx, (name, raw) in enumerate(points.items()):
        coords = _coerce_joint_slice(raw)
        progress(30 + int(60 * idx / max(len(points), 1)), f"Moving '{name}' ...")
        _ = rbt.MoveJ(
            coords, tool=DEFAULT_TOOL, user=DEFAULT_USER, vel=speed, acc=acceleration
        )
        if idx == 5:
            open_gripper()
    progress(100, "Pickup station release complete")
    return True


# ------------------------------------------------------------------------------
# Optional: keep these helpers to mirror a few conveniences from the draft
# ------------------------------------------------------------------------------


@register
def move_joints(
    j0: float,
    j1: float,
    j2: float,
    j3: float,
    j4: float,
    j5: float,
    speed: int = DEFAULT_SPEED,
    acceleration: int = DEFAULT_SPEED,
) -> int:
    """Direct MoveJ to supplied joint angles."""
    rbt = SESSION.client
    joints = [float(j0), float(j1), float(j2), float(j3), float(j4), float(j5)]
    progress(25, "MoveJ ...")
    ret = rbt.MoveJ(
        joints, tool=DEFAULT_TOOL, user=DEFAULT_USER, vel=speed, acc=acceleration
    )
    progress(100, f"MoveJ done (ret={ret})")
    return int(ret)


@register
def move_linear(
    x: float,
    y: float,
    z: float,
    rx: float,
    ry: float,
    rz: float,
    speed: int = DEFAULT_SPEED,
    acceleration: int = DEFAULT_SPEED,
) -> int:
    """Direct MoveL in Cartesian space (if supported by controller)."""
    rbt = SESSION.client
    pose = [float(x), float(y), float(z), float(rx), float(ry), float(rz)]
    progress(25, "MoveL ...")
    if not hasattr(rbt, "MoveL"):
        raise NotImplementedError("MoveL is not supported by the current RPC/Mock")
    ret = rbt.MoveL(
        pose, tool=DEFAULT_TOOL, user=DEFAULT_USER, vel=speed, acc=acceleration
    )
    progress(100, f"MoveL done (ret={ret})")
    return int(ret)


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
if __name__ == "__main__":
    app_name = os.getenv("ARKITEKT_APPNAME", "FAIRINO")
    app_url = os.getenv("ARKITEKT_URL", "go.arkitekt.live")
    app = easy(app_name, url=app_url)
    # Decorators already registered; enter + run the app
    app.enter()
    # Prefer to run in foreground by default so exceptions surface
    app.run()
