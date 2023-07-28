from typing import List

MOVE_GROUP_ARM: str = "right_arm"
MOVE_GROUP_GRIPPER: str = "gripper"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.04, 0.04]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]


def joint_names(prefix: str = "right_") -> List[str]:
    return [
        prefix + "e0",
        prefix + "e1",
        prefix + "s0",
        prefix + "s1",
        prefix + "w0",
        prefix + "w1",
        prefix + "w2",
    ]


def base_link_name(name: str = "base") -> str:
    return name


def end_effector_name(prefix: str = "right_") -> str:
    return prefix + "hand"


def gripper_joint_names(prefix: str = "panda_") -> List[str]:
    return [
        prefix + "finger_joint1",
        prefix + "finger_joint2",
    ]
