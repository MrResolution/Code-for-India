import xml.etree.ElementTree as ET

urdf_in = '/home/sabo/Documents/learn_/Hardware/urdf/unnamed/urdf/unnamed_gazebo.urdf'
urdf_out = '/home/sabo/Documents/learn_/Hardware/urdf/unnamed/urdf/unnamed_gazebo.urdf'

tree = ET.parse(urdf_in)
root = tree.getroot()

# Reset bad Phobos joint rotations for arm chain so vertical STL meshes stay upright
clean_rpy_joints = [
    'turntable_link_joint_dup',
    'turntable_link_joint',
    'turntable_link_joint_dup_1',
    'turntable_link_joint_dup_2',
    'turntable_link_joint_dup_3'
]

for joint in root.findall('joint'):
    jname = joint.get('name')
    if jname in clean_rpy_joints:
        orig = joint.find('origin')
        if orig is not None:
            orig.set('rpy', '0 0 0')
            # Ensure joint axis is aligned for rotation (e.g. Y axis for pitch joints)
            axis = joint.find('axis')
            if axis is not None:
                if jname == 'turntable_link_joint_dup':
                    axis.set('xyz', '0 0 1') # yaw
                elif jname in ('turntable_link_joint', 'turntable_link_joint_dup_1', 'turntable_link_joint_dup_2'):
                    axis.set('xyz', '0 1 0') # pitch
                elif jname == 'turntable_link_joint_dup_3':
                    axis.set('xyz', '1 0 0') # roll

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
print("Updated unnamed_gazebo.urdf: reset joint rpy orientations so arm links stand upright!")
