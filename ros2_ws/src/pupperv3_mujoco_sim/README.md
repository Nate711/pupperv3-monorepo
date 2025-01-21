# Prereqs
* ARM computer
* If you have a x86 computer. Download Mujoco 2.3.1 release https://github.com/google-deepmind/mujoco/releases/download/2.3.1/mujoco-2.3.1-linux-x86_64.tar.gz and replace the existing third_party/mujoco-2.3.1 directory.
* pupper_v3_description (https://github.com/G-Levine/pupper_v3_description) repo in the same ros workspace 

# Install
```bash
sudo apt install -y libglfw3-dev
cd ${YOUR_WORKSPACE_FOLDER}/src
git clone https://github.com/Nate711/pupperv3_mujoco_sim.git
source /opt/ros/humble/setup.bash
cd ${YOUR_WORKSPACE_FOLDER}
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1
source /install/local_setup.bash
```

# Usage
Using the simulator as a hardware interface
```bash
ros2 launch pupper_mujoco_sim test_hw_interface.launch.py
```

Using the simulator as a regular ros node with subscription to joint commands
```bash
ros2 launch pupper_mujoco_sim floating_base_launch.py 
```

# TODO / Limitations
* Make mujoco_core_interactive.hpp load model xml based on package and model parameter rather than hardcode to `pupper_v3_description/description/mujoco_xml/pupper_v3_complete.xml`
* Make hardware interface set model package and xml file based on parameter 
