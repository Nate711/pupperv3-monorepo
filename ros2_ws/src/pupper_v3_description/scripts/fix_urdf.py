import xml.etree.ElementTree as ET
import argparse
import pathlib


def fix_mesh_paths(urdf_file_path, output_file_path):
    # Parse the URDF file
    tree = ET.parse(urdf_file_path)
    root = tree.getroot()

    # Loop through all 'mesh' tags and fix the 'filename' attribute
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        # Check if the filename needs to be updated
        if filename.startswith("../"):
            # Update the filename
            new_filename = (
                ("package://pupper_v3_description/description/" + filename[3:])
                .replace(" ", "")
                .replace("dae", "stl")
            )
            new_filename = new_filename.rsplit(".", 1)
            new_filename[0] = new_filename[0].replace(".", "_")
            new_filename = ".".join(new_filename)
            mesh.set("filename", new_filename)

    # Add world body and joint between world and body
    world_to_robot_joint = ET.SubElement(root, "joint", name="world_to_body", type="floating")
    ET.SubElement(world_to_robot_joint, "parent", link="world")
    ET.SubElement(world_to_robot_joint, "child", link="base_link")
    world = ET.SubElement(root, "link", name="world")

    # Define the new mujoco tag and its child
    mujoco_tag = ET.Element("mujoco")
    compiler_tag = ET.SubElement(
        mujoco_tag,
        "compiler",
        attrib={"meshdir": "../meshes/stl/", "discardvisual": "false"},
    )

    # Insert the mujoco tag right below the robot tag
    root.insert(0, mujoco_tag)

    # Write the modified tree to a new file
    tree.write(output_file_path)


parser = argparse.ArgumentParser()
parser.add_argument("--urdf_path", type=pathlib.Path, required=True)
args = parser.parse_args()

fix_mesh_paths(args.urdf_path, args.urdf_path.with_suffix(".fixed.urdf"))
