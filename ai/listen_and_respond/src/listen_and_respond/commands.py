"""The unified command format: what ZhuangZi can be told to do physically.

This file is the single source of truth for the command vocabulary. It is independent of
the language model: understand.py turns it into a JSON schema for whichever model is used,
and robot.py turns each Command into ROS 2 calls on the running robot.

Movements are described semantically (direction, distance, angle), not as velocities.
The executor picks speeds within the robot's limits (carried over from the removed LiveKit agent):
forward/backward 0.4-0.75 m/s, sideways 0.4-0.5 m/s, turning 30-120 deg/s; slower is ignored
by the real robot.
"""

from typing import Literal

from pydantic import BaseModel, Field

Action = Literal["move", "turn", "stop", "stand_up", "lie_down", "trick"]

# Pre-recorded animations available in the animation controller
# (CSV files in ros2_ws/src/animation_controller_py/launch/animations).
Trick = Literal[
    "twerk",
    "lie_sit_lie",
    "stand_sit_shake_sit_stand",
    "upward_dog",
    "superman",
    "pee",
    "lie_downward_dog",
]


class Command(BaseModel):
    action: Action
    direction: Literal["forward", "backward", "left", "right"] | None = Field(
        description="move: which way to walk (left/right = sideways step). turn: left or right. Otherwise null."
    )
    distance_m: float | None = Field(
        description="move: distance in meters, null if the speaker gave none. Otherwise null."
    )
    angle_deg: float | None = Field(
        description="turn: angle in degrees (a full spin is 360), null if the speaker gave none. Otherwise null."
    )
    trick: Trick | None = Field(description="trick: which animation. Otherwise null.")


# Shown to the model in the system prompt, one line per action.
ACTION_DESCRIPTIONS = {
    "move": "walk forward, backward, or step sideways left/right, optionally a distance",
    "turn": "rotate in place left or right, optionally by an angle (spin around = 360)",
    "stop": "stop moving immediately and drop any queued commands",
    "stand_up": "power the legs and stand up in 4-legged walking mode, ready to walk (also for start walking, get up, wake up)",
    "lie_down": "lie down and relax the motors",
    "trick": "play a recorded animation",
}

TRICK_DESCRIPTIONS = {
    "twerk": "moves its hips rhythmically",
    "lie_sit_lie": "from lying, sits up then lies back down",
    "stand_sit_shake_sit_stand": "from standing, sits, shakes its body, then stands back up",
    "upward_dog": "from lying, upward-dog yoga pose and back down",
    "superman": "from lying, lifts arms and legs like flying",
    "pee": "from standing, lifts a leg like a dog peeing",
    "lie_downward_dog": "from lying, downward-dog yoga pose",
}


def describe_commands() -> str:
    actions = "\n".join(f"- {name}: {desc}" for name, desc in ACTION_DESCRIPTIONS.items())
    tricks = "\n".join(f"  - {name}: {desc}" for name, desc in TRICK_DESCRIPTIONS.items())
    return f"{actions}\nTricks:\n{tricks}"
