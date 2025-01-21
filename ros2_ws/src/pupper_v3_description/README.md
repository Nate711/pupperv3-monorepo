# Pupper v3
Description package for the Pupper v3 robot. Provides both URDF and XML files for visualization and simulation.

## How to make URDF and MJCF XML robot model files
0. Build this package
1. Export pupper v3 from CAD
2. Use Phobos Blender plugin to construct joints and bodies
3. Export to URDF using Phobos plugin. Select STL as mesh type.
4. Fix up URDF using `fix_urdf.py` script e.g. `python3 scripts/fix_urdf.py --urdf_path=/home/parallels/pupperv3_ws/src/pupper_v3_description/description/urdf/pupper_v3.edited.urdf`. Pass `--help` for all arguments.
5. Verify using RVIZ. `ros2 launch pupper_v3_description display_model.launch.py`
5. Convert to Mujoco XML by using compile or simulate executable. For simulate, run `ros2 run pupper_mujoco_sim simulate`, drag-and-drop the fixed urdf file into the window, click "Save xml" and copy to `pupper_v3_description/description/mujoco_xml`
6. Compose final Mujoco xml using `create_mujoco_xml.py` script: `python3 scripts/create_mujoco_xml.py --xml_dir=/home/parallels/pupperv3_ws/src/pupper_v3_description/description/mujoco_xml`. Pass `--mjx` for mjx-compatible xml. Pass `--fixed` for fixed-base model. Pass `--all` to generate xml for all permutations. Pass `--help` for more info.
7. Format the xml files using VSCode XML extension
8. Verify using Mujoco simulate executable. `ros2 run pupper_mujoco_sim simulate` and this time drag in the `pupper_v3_complete.xml` file.