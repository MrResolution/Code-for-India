import xml.etree.ElementTree as ET
import os

urdf_path = '/home/sabo/Documents/learn_/Hardware/urdf/unnamed/urdf/unnamed.urdf'
urdf_out = '/home/sabo/Documents/learn_/Hardware/urdf/unnamed/urdf/unnamed_gazebo.urdf'

tree = ET.parse(urdf_path)
root = tree.getroot()

# Exact scale factors per joint to match physical mesh pivot-to-pivot distances
exact_joint_scales = {
    "turntable_link_joint_dup": 0.107,          # Base -> Turntable
    "turntable_link_joint": 0.107,              # Turntable -> Elbow 1
    "turntable_link_joint_dup_1": 0.1071,       # Elbow 1 -> Elbow 2 (0.2141m / 1.999)
    "turntable_link_joint_dup_2": 0.1093,       # Elbow 2 -> Wrist (0.1529m / 1.398)
    "turntable_link_joint_dup_3": 0.1495,       # Wrist -> Gripper Base (0.0605m / 0.4044)
    "left_parallel_link_joint": 0.107,          # Gripper Base -> Left Parallel Rod
    "right_parallel_link_joint": 0.107,         # Gripper Base -> Right Parallel Rod
    "turntable_link_joint_dup_4": 0.107,        # Gripper Base -> Left Geared Rod
    "turntable_link_joint_dup_5": 0.107,        # Gripper Base -> Right Geared Rod
    "Gripper+finger+Geared_left_link": 0.1388,  # Left Geared Rod -> Left Finger (0.0425m / 0.3062)
    "Gripper+finger+Geared_right_link": 0.1388, # Right Geared Rod -> Right Finger (0.0431m / 0.3105)
    "Gripper_finger_Geared_left_joint": 0.1388,
    "Gripper_finger_Geared_right_joint": 0.1388,
}
default_scale = 0.107

# Apply exact joint origin scales
for joint in root.findall('joint'):
    jname = joint.get('name')
    if jname == 'fixed_base':
        continue
    orig = joint.find('origin')
    if orig is not None and 'xyz' in orig.attrib:
        raw_coords = [float(c) for c in orig.get('xyz').split()]
        scale_f = exact_joint_scales.get(jname, default_scale)
        scaled_coords = [c * scale_f for c in raw_coords]
        orig.set('xyz', f"{scaled_coords[0]:.6f} {scaled_coords[1]:.6f} {scaled_coords[2]:.6f}")

# Set correct scale for mg996r servo mesh (0.01 converts 8.3cm bounds to 0.083m)
for link in root.findall('link'):
    if link.get('name') == 'gripper_base_link':
        for m in link.findall('.//mesh'):
            m.set('scale', '0.0100 0.0100 -0.0100')

def indent(elem, level=0):
    i = "\n" + level*"  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for subelem in elem:
            indent(subelem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i

indent(root)
tree.write(urdf_out, encoding='utf-8', xml_declaration=True)
print("Updated unnamed_gazebo.urdf with exact joint pivot alignments!")
