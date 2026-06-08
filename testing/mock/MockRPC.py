import time
import json
import threading
from typing import List, Dict, Optional, Tuple


# Fallback RPC class for when Fairino SDK is not available
class MockRPC:
    """Mock implementation of Fairino RPC for testing/fallback purposes"""

    def __init__(self, ip: str):
        self.ip = ip
        self.is_enabled = False
        self.mode = 0
        self.current_user = 0
        self.current_tool = 0
        self.joint_positions = [0.0] * 6
        self.gripper_config: Dict[str, int] = {}
        self.gripper_active = False
        self.gripper_state = {
            "jawnumber": 1,
            "position": 0,
            "force": 0,
            "velocity": 0,
        }
        print(f"[MOCK] Connecting to robot at {ip}")

    def RobotEnable(self, enable: int) -> bool:
        """Enable or disable robot"""
        self.is_enabled = bool(enable)
        print(f"[MOCK] Robot {'enabled' if enable else 'disabled'}")
        return True

    def Mode(self, mode: int) -> bool:
        """Set robot mode"""
        self.mode = mode
        print(f"[MOCK] Mode set to {mode}")
        return True

    def User(self, user: int) -> bool:
        """Set user coordinate system"""
        self.current_user = user
        print(f"[MOCK] User coordinate system set to {user}")
        return True

    def Tool(self, tool: int) -> bool:
        """Set tool coordinate system"""
        self.current_tool = tool
        print(f"[MOCK] Tool coordinate system set to {tool}")
        return True

    def MoveJ(self, joints: List[float], tool: int, user: int, vel: int = 30, acc: int = 30) -> bool:
        """Move robot to joint position"""
        print(f"[MOCK] MoveJ: joints={joints}, tool={tool}, user={user}, vel={vel}, acc={acc}")
        self.joint_positions = list(joints)
        time.sleep(0.1)  # Simulate movement time
        return True

    def MoveL(self, pose: List[float], tool: int, user: int, vel: int = 30) -> bool:
        """Move robot linearly to cartesian position"""
        print(f"[MOCK] MoveL: pose={pose}, tool={tool}, user={user}, vel={vel}")
        time.sleep(0.1)  # Simulate movement time
        return True

    def Home(self) -> bool:
        """Move robot to home position"""
        print("[MOCK] Moving to home position")
        time.sleep(0.1)  # Simulate movement time
        return True

    def SetGripperConfig(
        self, company: int, device: int, softversion: int = 0, bus: int = 0
    ) -> int:
        """Mock gripper configuration"""
        self.gripper_config = {
            "company": company,
            "device": device,
            "softversion": softversion,
            "bus": bus,
        }
        print(f"[MOCK] Gripper configured: {self.gripper_config}")
        return 0

    def ActGripper(self, jawnumber: int, activate: int) -> int:
        """Mock gripper activation"""
        self.gripper_active = bool(activate)
        self.gripper_state["jawnumber"] = jawnumber
        state = "activated" if self.gripper_active else "deactivated"
        print(f"[MOCK] Gripper {state} for jaw {jawnumber}")
        return 0

    def MoveGripper(
        self,
        jawnumber: int,
        position: int,
        force: int,
        velocity: int,
        maxtime: int,
        is_blocking: int,
        typ: int,
        rot_num: int,
        rot_vel: int,
        rot_torque: int,
    ) -> int:
        """Mock gripper movement"""
        self.gripper_state.update(
            {
                "jawnumber": jawnumber,
                "position": position,
                "force": force,
                "velocity": velocity,
                "maxtime": maxtime,
                "is_blocking": is_blocking,
                "typ": typ,
                "rot_num": rot_num,
                "rot_vel": rot_vel,
                "rot_torque": rot_torque,
            }
        )
        print(f"[MOCK] Gripper move: {self.gripper_state}")
        time.sleep(0.05)
        return 0

    def GetActualJointPosDegree(self) -> Tuple[int, List[float]]:
        """Return mock joint positions"""
        print(f"[MOCK] Reporting joint positions: {self.joint_positions}")
        return 0, list(self.joint_positions)
