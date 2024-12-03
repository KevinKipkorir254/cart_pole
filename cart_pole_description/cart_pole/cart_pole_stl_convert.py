import os
import trimesh
from xml.etree import ElementTree as ET

# Function to convert STL to OBJ
def convert_stl_to_obj(stl_path, obj_path):
    if not os.path.exists(stl_path):
        raise FileNotFoundError(f"STL file not found: {stl_path}")
    mesh = trimesh.load_mesh(stl_path)
    mesh.export(obj_path)
    print(f"Converted {stl_path} to {obj_path}")

# Update URDF file paths
def update_urdf(urdf_path, output_dir, package_root):
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # Iterate through mesh filenames in the URDF
    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib["filename"]
        if filename.startswith("package://"):
            # Convert package:// to actual path
            relative_path = filename.replace("package://", "")
            stl_path = os.path.join(package_root, relative_path)
            obj_path = os.path.join(output_dir, os.path.basename(stl_path).replace(".stl", ".obj"))

            # Convert STL to OBJ
            convert_stl_to_obj(stl_path, obj_path)

            # Update the filename in URDF
            new_filename = filename.replace(".stl", ".obj")
            mesh.attrib["filename"] = new_filename

    # Save updated URDF to the current directory
    updated_urdf_path = os.path.join(output_dir, os.path.basename(urdf_path))
    tree.write(updated_urdf_path)
    print(f"Updated URDF saved to {updated_urdf_path}")

# Paths
urdf_path = "/home/kevo/robotics_inc/under_actuated_ststems/cart_pole_ws/src/cart_pole/cart_pole_description/cart_pole/robot.urdf"
package_root = "/home/kevo/robotics_inc/under_actuated_ststems/cart_pole_ws/src/cart_pole"  # Replace with your package root directory
output_dir = os.getcwd()  # Set output directory to the current working directory

# Ensure output directory exists (it always does with os.getcwd())
os.makedirs(output_dir, exist_ok=True)

# Update URDF and convert STL to OBJ
update_urdf(urdf_path, output_dir, package_root)
