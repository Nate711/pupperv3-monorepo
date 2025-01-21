# real2sim_controller
This repository aims to ease the deployment of neural network control policies on robot hardware. It runs within the [ros2_control](https://github.com/ros-controls/ros2_control) framework and adopts [RTNeural](https://github.com/jatinchowdhury18/RTNeural) for real-time inference.

## Motivation
Getting a policy up and running on real hardware can be surprisingly tricky. Many opportunities exist for mistakes in processing the observations and actions. 
Furthermore, common neural network libraries such as Torch are not designed to run in a real-time control loop, introducing latency/jitter that increases the difficulty of sim-to-real transfer. 

# Install
`git clone git@github.com:Nate711/real2sim_controller.git`

# Functionality
## Inputs
- Maps hardware_interface states and ROS topics (such as cmd_vel) to the policy obersvation vector
- Performs processing such as quaternion-to-gravity-vector conversion and scaling/normalization on the observations
- Allows for parts of the observation vector to be hardcoded as fixed values

## Outputs
- Performs scaling/normalization on the actions
- Maps the policy action vector to hardware_interface commands

## Convenience and safety features
- On startup, smoothly returns the robot to a predefined starting pose
- Triggers an emergency stop when safety limits are exceeded (joint velocity, body pitch, etc.)

## How to configure
```
# Names of the joints in the order expected by the policy
joint_names:
    - "left_hip"
    - "left_knee"
    - "left_wheel"
    - "right_hip"
    - "right_knee"
    - "right_wheel"

# Position gains of the joints
kps:
    left_hip: 20.0
    left_knee: 20.0
    left_wheel: 0.0
    right_hip: 20.0
    right_knee: 20.0
    right_wheel: 0.0

# Velocity gains of the joints
kds:
    left_hip: 2.0
    left_knee: 2.0
    left_wheel: 0.5
    right_hip: 2.0
    right_knee: 2.0
    right_wheel: 0.5

# Type of action for each joint (position or velocity)
action_types:
    left_hip: "P"
    left_knee: "P"
    left_wheel: "V"
    right_hip: "P"
    right_knee: "P"
    right_wheel: "V"

# Center of the observation/action range for each joint
default_joint_pos:
    left_hip: 2.0
    left_knee: 2.0
    left_wheel: 0.0
    right_hip: 2.0
    right_knee: 2.0
    right_wheel: 0.0

# Observation and action scaling factors
obs_ang_vel_scale: 1.0

obs_joint_pos_scale: 1.0
obs_joint_vel_scale: 1.0

action_joint_pos_scale: 1.0
action_joint_vel_scale: 1.0

cmd_lin_vel_scale: 1.0
cmd_ang_vel_scale: 1.0
```