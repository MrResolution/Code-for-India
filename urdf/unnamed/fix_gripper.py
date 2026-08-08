import xml.etree.ElementTree as ET

urdf_path = '/home/sabo/Documents/learn_/Hardware/urdf/unnamed/urdf/unnamed_gazebo.urdf'
urdf_out = '/home/sabo/Documents/learn_/Hardware/urdf/unnamed/urdf/unnamed_gazebo.urdf'

tree = ET.parse(urdf_path)
root = tree.getroot()

# Fix 1: Scale mg996r servo mesh by 0.01 (so 8.3 units becomes 8.3 cm instead of 0.8 cm)
for link in root.findall('link'):
    if link.get('name') == 'gripper_base_link':
        for tag in link.findall('.//mesh'):
            tag.set('scale', '0.0100 0.0100 -0.0100')

# Fix 2: Adjust gripper joint origins attached to gripper_base_link
# In raw URDF, gripper joints had X ~ -0.66, Y ~ 0.08, Z ~ 0.09.
# With scale 0.1, X was -0.066m. With scale 0.03, X becomes -0.02 m (-2cm), attaching properly to servo.
gripper_joints = [
    'left_parallel_link_joint',
    'right_parallel_link_joint',
    'turntable_link_joint_dup_4',
    'turntable_link_joint_dup_5',
    'Gripper_finger_Geared_left_joint',
    'Gripper_finger_Geared_right_joint',
    'Gripper+finger+Geared_left_link',
    'Gripper+finger+Geared_right_link'
]

# Raw URDF values to reference
raw_gripper_origins = {
    'left_parallel_link_joint': (-0.51914, -0.23759, 0.09017),
    'right_parallel_link_joint': (-0.51914, 0.24594, 0.09017),
    'turntable_link_joint_dup_4': (-0.66831, -0.07284, 0.09054),
    'turntable_link_joint_dup_5': (-0.66831, 0.08119, 0.09054),
    'Gripper_finger_Geared_left_joint': (-0.30120, 0.00058, -0.05526),
    'Gripper_finger_Geared_right_joint': (0.30158, -0.00038, -0.07414)
}

SCALE_GRIPPER_JOINTS = 0.04  # 4cm offset matching servo dimensions

for joint in root.findall('joint'):
    jname = joint.get('name')
    if jname in raw_gripper_origins:
        raw_xyz = raw_gripper_origins[jname]
        scaled = [c * SCALE_GRIPPER_JOINTS for c in raw_xyz]
        orig = joint.find('origin')
        if orig is not None:
            orig.set('xyz', f"{scaled[0]:.6f} {scaled[1]:.6f} {scaled[2]:.6f}")

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
print("Updated gripper base mesh scale and gripper joint origins in unnamed_gazebo.urdf")
