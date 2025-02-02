# Prerequisites
Give everyone access to Pupper's screen:

`sudo chmod a+w /dev/tty1`

# Install
In the workspace dir (`~/ros2_ws`) 

`colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`

# run
1) `ros2 run joy_linux joy_linux_node --ros-args -p coalesce_interval:=0.02`
2)  `ros2 run pupper_feelings face_control`