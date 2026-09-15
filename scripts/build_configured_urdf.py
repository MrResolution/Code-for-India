import os
import xml.etree.ElementTree as ET
from xml.dom import minidom

def generate_configured_urdf(output_path, mesh_dir_abs):
    robot = ET.Element('robot', {'name': 'quard_bot'})

    # Materials
    materials = [
        ('teal_chassis', '0.35 0.65 0.58 1.0'),
        ('blue_r1',       '0.38 0.55 0.72 1.0'),
        ('orange_r3',     '0.72 0.50 0.40 1.0'),
        ('green_r2',      '0.45 0.68 0.50 1.0'),
        ('purple_hip',    '0.62 0.44 0.62 1.0'),
        ('purple_foot',   '0.50 0.42 0.58 1.0'),
        ('top_cover_mat', '0.28 0.55 0.50 1.0')
    ]
    for name, rgba in materials:
        m = ET.SubElement(robot, 'material', {'name': name})
        ET.SubElement(m, 'color', {'rgba': rgba})

    # Scale factor: exported unnamed.urdf has origins in centimeters -> convert to meters
    SCALE = 0.01

    # Exact joint data matching Blender armatures
    joints_data = [
        ('joint_l1_hip',  'revolute', 'Internal-Frame-v121_link', 'L1-v117_link',
         (2.46389, 2.58302, 2.03634), (0.0, 0.0, 0.0), (0, 0, 1), -0.7854, 0.7854),
        ('joint_l3_foot', 'revolute', 'L1-v117_link', 'L3-v117_link',
         (-0.93384, 3.68844, -1.95242), (1.5708, 0.0, 1.5708), (0, 0, 1), -0.7854, 1.0472),
        ('joint_l2_hip',  'revolute', 'Internal-Frame-v121_link', 'L2-v117_link',
         (-2.39531, 2.58330, 2.03634), (0.0, 0.0, 3.14159), (0, 0, 1), -0.7854, 0.7854),
        ('joint_l4_foot', 'revolute', 'L2-v117_link', 'L4-v117_link',
         (-0.93494, -3.68698, -1.95475), (1.55975, 0.0, 1.5708), (0, 0, 1), -0.7854, 1.0472),
        ('joint_r1_hip',  'revolute', 'Internal-Frame-v121_link', 'R1-v117_link',
         (2.46494, -2.51056, 2.03634), (0.0, 0.0, 0.0), (0, 0, 1), -0.7854, 0.7854),
        ('joint_r3_foot', 'revolute', 'R1-v117_link', 'R3-v117_link',
         (-0.94144, -3.68561, -1.95306), (1.55156, 0.0, 1.5708), (0, 0, 1), -0.7854, 1.0472),
        ('joint_r2_hip',  'revolute', 'Internal-Frame-v121_link', 'R2-v117_link',
         (-2.40342, -2.49865, 2.03634), (0.0, 0.0, 3.14159), (0, 0, 1), -0.7854, 0.7854),
        ('joint_r4_foot', 'revolute', 'R2-v117_link', 'R4-v117_link',
         (-0.95498, 3.67349, -1.95620), (1.55156, 0.0, 1.5708), (0, 0, 1), -0.7854, 1.0472)
    ]

    for name, jtype, parent, child, xyz_cm, rpy, axis, lower, upper in joints_data:
        j = ET.SubElement(robot, 'joint', {'name': name, 'type': jtype})
        ET.SubElement(j, 'parent', {'link': parent})
        ET.SubElement(j, 'child', {'link': child})
        xyz_m = f"{xyz_cm[0]*SCALE:.6f} {xyz_cm[1]*SCALE:.6f} {xyz_cm[2]*SCALE:.6f}"
        rpy_str = f"{rpy[0]:.5f} {rpy[1]:.5f} {rpy[2]:.5f}"
        ET.SubElement(j, 'origin', {'xyz': xyz_m, 'rpy': rpy_str})
        ET.SubElement(j, 'axis', {'xyz': f"{axis[0]} {axis[1]} {axis[2]}"})
        ET.SubElement(j, 'limit', {
            'lower': f"{lower:.4f}",
            'upper': f"{upper:.4f}",
            'effort': "2.5",
            'velocity': "6.0"
        })

    # Link specifications (name, mesh_file, material_name, mass, (ixx, iyy, izz))
    links_data = [
        ('Internal-Frame-v121_link', 'chassis.stl', 'teal_chassis', 0.28, (0.00018, 0.00022, 0.00030)),
        ('L1-v117_link',            'L1.stl',      'purple_hip',   0.03, (0.00001, 0.00001, 0.00001)),
        ('L3-v117_link',            'L3.stl',      'purple_foot',  0.02, (0.00001, 0.00001, 0.00001)),
        ('L2-v117_link',            'L2.stl',      'purple_hip',   0.03, (0.00001, 0.00001, 0.00001)),
        ('L4-v117_link',            'L4.stl',      'purple_foot',  0.02, (0.00001, 0.00001, 0.00001)),
        ('R1-v117_link',            'R1.stl',      'blue_r1',      0.03, (0.00001, 0.00001, 0.00001)),
        ('R3-v117_link',            'R3.stl',      'orange_r3',    0.02, (0.00001, 0.00001, 0.00001)),
        ('R2-v117_link',            'R2.stl',      'green_r2',     0.03, (0.00001, 0.00001, 0.00001)),
        ('R4-v117_link',            'R4.stl',      'purple_foot',  0.02, (0.00001, 0.00001, 0.00001)),
    ]

    for link_name, mesh_file, mat_name, mass, (ixx, iyy, izz) in links_data:
        link = ET.SubElement(robot, 'link', {'name': link_name})

        # Inertial
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass', {'value': str(mass)})
        ET.SubElement(inertial, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
        ET.SubElement(inertial, 'inertia', {
            'ixx': str(ixx), 'ixy': '0', 'ixz': '0',
            'iyy': str(iyy), 'iyz': '0', 'izz': str(izz)
        })

        mesh_uri = f"file://{os.path.join(mesh_dir_abs, mesh_file)}"

        # Visual
        visual = ET.SubElement(link, 'visual')
        ET.SubElement(visual, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
        geom = ET.SubElement(visual, 'geometry')
        ET.SubElement(geom, 'mesh', {'filename': mesh_uri, 'scale': '1 1 1'})
        mat_elem = ET.SubElement(visual, 'material', {'name': mat_name})
        mat_dict = dict(materials)
        ET.SubElement(mat_elem, 'color', {'rgba': mat_dict[mat_name]})

        # Collision
        collision = ET.SubElement(link, 'collision')
        ET.SubElement(collision, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
        geom_c = ET.SubElement(collision, 'geometry')
        ET.SubElement(geom_c, 'mesh', {'filename': mesh_uri, 'scale': '1 1 1'})

    # Prettify XML
    raw_xml = ET.tostring(robot, encoding='utf-8')
    parsed = minidom.parseString(raw_xml)
    pretty_xml = parsed.toprettyxml(indent='  ')
    lines = [l for l in pretty_xml.splitlines() if l.strip()]
    cleaned_xml = '\n'.join(lines) + '\n'

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(cleaned_xml)
    print(f"✅ Generated updated configured URDF: {output_path}")


def generate_configured_arm_urdf(output_path, mesh_dir_abs):
    """
    Generate configured 5-DOF robot arm URDF without claw grip action and armature.
    Origins scaled by 0.1 (Blender decimeter to meter conversion).
    Meshes scaled by 0.001 (STL millimeter to meter conversion).
    """
    robot = ET.Element('robot', {'name': 'arm', 'version': '1.0'})

    # Materials
    materials = [
        ('DarkSteel',      '0.22 0.24 0.28 1.0'),
        ('Copper',         '0.72 0.45 0.20 1.0'),
        ('Teal',           '0.10 0.56 0.62 1.0'),
        ('Aluminum',       '0.75 0.75 0.80 1.0'),
        ('AccentOrange',   '0.90 0.45 0.10 1.0'),
        ('Gunmetal',       '0.32 0.34 0.36 1.0'),
        ('MatteBlack',     '0.15 0.15 0.15 1.0'),
        ('IndustrialBlue', '0.00 0.45 0.75 1.0'),
    ]
    for name, rgba in materials:
        m = ET.SubElement(robot, 'material', {'name': name})
        ET.SubElement(m, 'color', {'rgba': rgba})

    # Fixed base to world
    ET.SubElement(robot, 'link', {'name': 'world'})
    fixed_joint = ET.SubElement(robot, 'joint', {'name': 'fixed_base', 'type': 'fixed'})
    ET.SubElement(fixed_joint, 'parent', {'link': 'world'})
    ET.SubElement(fixed_joint, 'child', {'link': 'basecase1_link'})
    ET.SubElement(fixed_joint, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})

    # 0.1 scale factor for Phobos decimeter coordinates -> meters
    SCALE = 0.1

    # Clean coaxial joint orientations matching physical CAD assembly
    joints_data = [
        ('turntable_link_joint_dup', 'revolute', 'basecase1_link', 'turntable_link',
         (-0.45306, -0.00297, 0.45118), (0.00000, 0.00000, 0.00443), (0.0, 0.0, 1.0), -3.0, 3.0),
        ('turntable_link_joint', 'revolute', 'turntable_link', 'Elbow_1_link',
         (-0.29849, -0.00704, 0.45105), (0.00000, 0.00000, 3.13716), (0.0, 1.0, 0.0), -2.0, 2.0),
        ('turntable_link_joint_dup_1', 'revolute', 'Elbow_1_link', 'Elbow_2_link',
         (0.47192, -0.01837, 1.94238), (1.57078, 1.56581, 1.57078), (0.0, 1.0, 0.0), -2.0, 2.0),
        ('turntable_link_joint_dup_2', 'revolute', 'Elbow_2_link', 'Wrist_pitch._link',
         (-0.00145, -0.00782, 1.39796), (0.00167, 0.01634, 0.01070), (0.0, 1.0, 0.0), -2.0, 2.0),
        ('turntable_link_joint_dup_3', 'revolute', 'Wrist_pitch._link', 'gripper_base_link',
         (-0.01615, 0.00642, 0.40411), (0.00000, 0.00000, 0.00000), (0.0, 0.0, 1.0), -3.14159, 3.14159),
    ]

    for name, jtype, parent, child, xyz_raw, rpy, axis, lower, upper in joints_data:
        j = ET.SubElement(robot, 'joint', {'name': name, 'type': jtype})
        ET.SubElement(j, 'parent', {'link': parent})
        ET.SubElement(j, 'child', {'link': child})
        xyz_m = f"{xyz_raw[0]*SCALE:.6f} {xyz_raw[1]*SCALE:.6f} {xyz_raw[2]*SCALE:.6f}"
        rpy_str = f"{rpy[0]:.5f} {rpy[1]:.5f} {rpy[2]:.5f}"
        ET.SubElement(j, 'origin', {'xyz': xyz_m, 'rpy': rpy_str})
        ET.SubElement(j, 'axis', {'xyz': f"{axis[0]:.1f} {axis[1]:.1f} {axis[2]:.1f}"})
        ET.SubElement(j, 'limit', {
            'lower': f"{lower:.4f}",
            'upper': f"{upper:.4f}",
            'effort': "30.0",
            'velocity': "2.0"
        })

    # 6 links matching re-exported arm URDF
    # (link_name, mesh_file, mat_name, scale_str, (xyz_vis), (rpy_vis), mass, (ixx, iyy, izz))
    links_data = [
        ('basecase1_link', 'part_3_obj3.stl', 'DarkSteel',
         '0.00100 0.00100 0.00100', (0, 0, 0), (0, 0, 0),
         2.50, (0.004021, 0.010207, 0.012187)),
        ('turntable_link', 'part_44_obj44.stl', 'MatteBlack',
         '0.00100 0.00100 0.00100', (0, 0, 0), (-1.04042, -1.54565, -0.52117),
         1.20, (0.002993, 0.003436, 0.003329)),
        ('Elbow_1_link', 'part_7_obj7.stl', 'IndustrialBlue',
         '0.00100 0.00100 0.00100', (0, 0, 0), (0, 0, 0),
         1.00, (0.005000, 0.005000, 0.005000)),
        ('Elbow_2_link', 'part_9_obj9.stl', 'Aluminum',
         '0.00100 0.00100 0.00100', (0, 0, 0), (0, 0, 0),
         0.80, (0.005000, 0.005000, 0.005000)),
        ('Wrist_pitch._link', 'Robot+Arm+Wrist.stl', 'AccentOrange',
         '0.00100 0.00100 0.00100', (0, 0, 0), (0, 0, 0),
         0.50, (0.000397, 0.000193, 0.000286)),
        ('gripper_base_link', 'gripper_base.stl', 'Gunmetal',
         '0.00100 0.00100 0.00100', (0, 0, 0), (0, 0, 0),
         0.30, (0.000100, 0.000100, 0.000100)),
    ]

    for link_name, mesh_file, mat_name, scale_str, (vx, vy, vz), (vr, vp, vyaw), mass, (ixx, iyy, izz) in links_data:
        link = ET.SubElement(robot, 'link', {'name': link_name})

        # Inertial
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass', {'value': f"{mass:.3f}"})
        ET.SubElement(inertial, 'origin', {'xyz': f"{vx} {vy} {vz}", 'rpy': f"{vr} {vp} {vyaw}"})
        ET.SubElement(inertial, 'inertia', {
            'ixx': f"{ixx:.6f}", 'ixy': '0', 'ixz': '0',
            'iyy': f"{iyy:.6f}", 'iyz': '0', 'izz': f"{izz:.6f}"
        })

        mesh_uri = f"file://{os.path.join(mesh_dir_abs, mesh_file)}"

        # Visual
        visual = ET.SubElement(link, 'visual', {'name': link_name.replace('_link', '')})
        ET.SubElement(visual, 'origin', {'xyz': f"{vx} {vy} {vz}", 'rpy': f"{vr} {vp} {vyaw}"})
        geom = ET.SubElement(visual, 'geometry')
        ET.SubElement(geom, 'mesh', {'filename': mesh_uri, 'scale': scale_str})
        mat_elem = ET.SubElement(visual, 'material', {'name': mat_name})
        mat_dict = dict(materials)
        ET.SubElement(mat_elem, 'color', {'rgba': mat_dict[mat_name]})

        # Collision
        collision = ET.SubElement(link, 'collision', {'name': f"{link_name}_col"})
        ET.SubElement(collision, 'origin', {'xyz': f"{vx} {vy} {vz}", 'rpy': f"{vr} {vp} {vyaw}"})
        geom_c = ET.SubElement(collision, 'geometry')
        ET.SubElement(geom_c, 'mesh', {'filename': mesh_uri, 'scale': scale_str})

    raw_xml = ET.tostring(robot, encoding='utf-8')
    parsed = minidom.parseString(raw_xml)
    pretty_xml = parsed.toprettyxml(indent='  ')
    lines = [l for l in pretty_xml.splitlines() if l.strip()]
    cleaned_xml = '\n'.join(lines) + '\n'

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(cleaned_xml)
    print(f"✅ Generated updated configured arm URDF: {output_path}")


if __name__ == '__main__':
    # 1. Quard Bot
    generate_configured_urdf(
        '/home/sabo/Documents/learn_/Hardware/urdf/quard_bot/urdf/quard_bot.urdf',
        '/home/sabo/Documents/learn_/Hardware/urdf/quard_bot/meshes'
    )
    # 2. Arm
    generate_configured_arm_urdf(
        '/home/sabo/Documents/learn_/Hardware/urdf/arm/urdf/arm.urdf',
        '/home/sabo/Documents/learn_/Hardware/urdf/arm/meshes/stl'
    )
