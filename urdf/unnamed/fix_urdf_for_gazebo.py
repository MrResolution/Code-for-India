import xml.etree.ElementTree as ET
import os
import struct

urdf_path = '/home/sabo/Documents/learn_/Hardware/urdf/unnamed/urdf/unnamed.urdf'
stl_dir = '/home/sabo/Documents/learn_/Hardware/urdf/unnamed/meshes/stl'
out_path = '/home/sabo/Documents/learn_/Hardware/urdf/unnamed/urdf/unnamed_gazebo.urdf'

def get_stl_bounds(filepath):
    if not os.path.exists(filepath):
        return (0.1, 0.1, 0.1)
    try:
        with open(filepath, 'rb') as f:
            header = f.read(80)
            count = struct.unpack('<I', f.read(4))[0]
            min_b = [float('inf')]*3
            max_b = [float('-inf')]*3
            for _ in range(count):
                f.read(12) # normal
                v1 = struct.unpack('<3f', f.read(12))
                v2 = struct.unpack('<3f', f.read(12))
                v3 = struct.unpack('<3f', f.read(12))
                f.read(2)
                for v in (v1, v2, v3):
                    for i in range(3):
                        min_b[i] = min(min_b[i], v[i])
                        max_b[i] = max(max_b[i], v[i])
            dx = abs(max_b[0] - min_b[0])
            dy = abs(max_b[1] - min_b[1])
            dz = abs(max_b[2] - min_b[2])
            return (dx, dy, dz)
    except Exception as e:
        print(f"Error reading {filepath}: {e}")
        return (0.1, 0.1, 0.1)

tree = ET.parse(urdf_path)
root = tree.getroot()

mass_map = {
    "basecase1_link": 2.5,
    "turntable_link": 1.2,
    "Elbow_1_link": 1.0,
    "Elbow_2_link": 0.8,
    "Wrist_pitch._link": 0.5,
    "gripper_base_link": 0.4,
}
default_mass = 0.1

# Define default material None
mat_none = ET.Element('material', {'name': 'None'})
ET.SubElement(mat_none, 'color', {'rgba': '0.6 0.6 0.6 1.0'})
root.insert(0, mat_none)

# Add world link and joint at the top
world_link = ET.Element('link', {'name': 'world'})
fixed_joint = ET.Element('joint', {'name': 'fixed_base', 'type': 'fixed'})
ET.SubElement(fixed_joint, 'parent', {'link': 'world'})
ET.SubElement(fixed_joint, 'child', {'link': 'basecase1_link'})
ET.SubElement(fixed_joint, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})

root.insert(1, world_link)
root.insert(2, fixed_joint)

# Collect link names
link_names = set(l.get('name') for l in root.findall('link'))
link_names.add('world')

# Update joints and resolve name collisions
used_joint_names = set()
for joint in root.findall('joint'):
    jname = joint.get('name', 'unnamed_joint')
    
    # If joint name is identical to a link name or already used, rename it
    if jname in link_names or jname in used_joint_names:
        if jname.endswith('_link'):
            new_jname = jname[:-5] + '_joint'
        else:
            new_jname = jname + '_joint'
        joint.set('name', new_jname)
        jname = new_jname
    used_joint_names.add(jname)

# Exact joint translation scale factors calculated from STL mesh lengths
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
    "Gripper+finger+Geared_left_link": 0.1388,  # Left Geared Rod -> Left Finger
    "Gripper+finger+Geared_right_link": 0.1388, # Right Geared Rod -> Right Finger
    "Gripper_finger_Geared_left_joint": 0.1388,
    "Gripper_finger_Geared_right_joint": 0.1388,
}
default_scale = 0.107

# Clean joint orientation RPY for main arm chain
clean_rpy_joints = [
    'turntable_link_joint_dup',
    'turntable_link_joint',
    'turntable_link_joint_dup_1',
    'turntable_link_joint_dup_2',
    'turntable_link_joint_dup_3'
]

for joint in root.findall('joint'):
    jname = joint.get('name')
    if jname == 'fixed_base':
        continue
    orig = joint.find('origin')
    if orig is not None:
        if jname in clean_rpy_joints:
            orig.set('rpy', '0 0 0')
        if 'xyz' in orig.attrib:
            raw_coords = [float(c) for c in orig.get('xyz').split()]
            scale_f = exact_joint_scales.get(jname, default_scale)
            scaled_coords = [c * scale_f for c in raw_coords]
            orig.set('xyz', f"{scaled_coords[0]:.6f} {scaled_coords[1]:.6f} {scaled_coords[2]:.6f}")

    # Set clean rotation axes for joint motion
    axis = joint.find('axis')
    if axis is not None:
        if jname == 'turntable_link_joint_dup':
            axis.set('xyz', '0 0 1') # yaw
        elif jname in ('turntable_link_joint', 'turntable_link_joint_dup_1', 'turntable_link_joint_dup_2'):
            axis.set('xyz', '0 1 0') # pitch
        elif jname == 'turntable_link_joint_dup_3':
            axis.set('xyz', '1 0 0') # roll

    jtype = joint.get('type')
    limit = joint.find('limit')
    if jtype in ('revolute', 'prismatic'):
        if limit is None:
            limit = ET.Element('limit', {'lower': '-1.57', 'upper': '1.57', 'effort': '30.0', 'velocity': '2.0'})
            joint.append(limit)
        else:
            eff = float(limit.get('effort', 0))
            vel = float(limit.get('velocity', 0))
            if eff <= 0:
                limit.set('effort', '30.0')
            if vel <= 0:
                limit.set('velocity', '2.0')

# Update links
for link in root.findall('link'):
    link_name = link.get('name')
    if link_name == 'world':
        continue
    
    # Scale correction for gripper_base_link servo mesh (mg996r-v17.005.stl was modeled in cm, so scale needs to be 0.01)
    if link_name == 'gripper_base_link':
        for m in link.findall('.//mesh'):
            m.set('scale', '0.0100 0.0100 -0.0100')
    
    visual = link.find('visual')
    if visual is not None:
        # Add collision if not existing
        if link.find('collision') is None:
            collision = ET.Element('collision', {'name': visual.get('name', link_name) + '_collision'})
            
            # Copy origin
            v_orig = visual.find('origin')
            if v_orig is not None:
                c_orig = ET.Element('origin', v_orig.attrib)
                collision.append(c_orig)
                
            # Copy geometry
            v_geom = visual.find('geometry')
            if v_geom is not None:
                c_geom = ET.Element('geometry')
                v_mesh = v_geom.find('mesh')
                if v_mesh is not None:
                    c_mesh = ET.Element('mesh', v_mesh.attrib)
                    c_geom.append(c_mesh)
                collision.append(c_geom)
            link.append(collision)
            
        # Add inertial if missing
        if link.find('inertial') is None:
            m_val = mass_map.get(link_name, default_mass)
            
            # estimate size from mesh
            v_mesh = visual.find('geometry/mesh')
            dx, dy, dz = 0.1, 0.1, 0.1
            if v_mesh is not None:
                filename = v_mesh.get('filename', '')
                scale_str = v_mesh.get('scale', '1 1 1')
                scales = [abs(float(s)) for s in scale_str.split()]
                if len(scales) == 3:
                    scale_x, scale_y, scale_z = scales
                else:
                    scale_x = scale_y = scale_z = 0.001
                
                # convert file:/// URI to local path
                clean_path = filename.replace('file://', '')
                raw_dx, raw_dy, raw_dz = get_stl_bounds(clean_path)
                dx = max(raw_dx * scale_x, 0.01)
                dy = max(raw_dy * scale_y, 0.01)
                dz = max(raw_dz * scale_z, 0.01)
            
            # Box inertia formula
            ixx = max(1/12.0 * m_val * (dy**2 + dz**2), 1e-4)
            iyy = max(1/12.0 * m_val * (dx**2 + dz**2), 1e-4)
            izz = max(1/12.0 * m_val * (dx**2 + dy**2), 1e-4)
            
            inertial = ET.Element('inertial')
            # Origin
            v_orig = visual.find('origin')
            if v_orig is not None:
                inertial.append(ET.Element('origin', v_orig.attrib))
            else:
                inertial.append(ET.Element('origin', {'xyz': '0 0 0', 'rpy': '0 0 0'}))
                
            inertial.append(ET.Element('mass', {'value': f'{m_val:.3f}'}))
            inertial.append(ET.Element('inertia', {
                'ixx': f'{ixx:.6f}', 'ixy': '0.000000', 'ixz': '0.000000',
                'iyy': f'{iyy:.6f}', 'iyz': '0.000000', 'izz': f'{izz:.6f}'
            }))
            link.append(inertial)

# Format and save XML
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
tree.write(out_path, encoding='utf-8', xml_declaration=True)
print(f"Generated gazebo-ready URDF at {out_path}")
