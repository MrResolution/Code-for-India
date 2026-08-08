import xml.etree.ElementTree as ET

urdf_in  = '/home/sabo/Documents/learn_/Hardware/urdf/unnamed/urdf/unnamed.urdf'
urdf_out = '/home/sabo/Documents/learn_/Hardware/urdf/unnamed/urdf/unnamed_gazebo.urdf'

tree = ET.parse(urdf_in)
root = tree.getroot()

# Links to remove (gripper assembly)
gripper_links = {
    'gripper_base_link',
    'left_parallel_link',
    'right_parallel_link',
    'Gripper+Rod+Geared_left_link',
    'Gripper+finger+Geared_left_link',
    'Gripper+Rod+Geared_right_link',
    'Gripper+finger+Geared_right_link'
}

for lk in list(root.findall('link')):
    if lk.get('name') in gripper_links:
        root.remove(lk)

for j in list(root.findall('joint')):
    parent = j.find('parent')
    child = j.find('child')
    p_name = parent.get('link') if parent is not None else ''
    c_name = child.get('link') if child is not None else ''
    if p_name in gripper_links or c_name in gripper_links:
        root.remove(j)

# Material definition
m = ET.Element('material', {'name': 'None'})
ET.SubElement(m, 'color', {'rgba': '0.6 0.6 0.6 1.0'})
root.insert(0, m)

# World link + fixed base
root.insert(1, ET.Element('link', {'name': 'world'}))
fj = ET.Element('joint', {'name': 'fixed_base', 'type': 'fixed'})
ET.SubElement(fj, 'parent', {'link': 'world'})
ET.SubElement(fj, 'child',  {'link': 'basecase1_link'})
ET.SubElement(fj, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
root.insert(2, fj)

# Fix joint/link name collisions
link_names = {l.get('name') for l in root.findall('link')}
used = set()
for j in root.findall('joint'):
    n = j.get('name', '')
    if n in link_names or n in used:
        new = (n[:-5]+'_joint') if n.endswith('_link') else (n+'_joint')
        j.set('name', new); n = new
    used.add(n)

# Coaxial joint origins - use original Phobos rpy rotations for correct mesh orientation
# xyz values are original Phobos values / 10 (Phobos unit scaling)
clean_origins = {
    'turntable_link_joint_dup':   ('-0.045306 -0.000297 0.045118', '0 0 0.00443',             '0 0 1'), # Base -> Turntable (Yaw)
    'turntable_link_joint':       ('-0.029849 -0.000704 0.045105', '0 0 3.13716',             '0 1 0'), # Turntable -> Elbow 1 (Pitch) - π around Z
    'turntable_link_joint_dup_1': ('0.047192 -0.001837 0.194238',  '1.57078 1.56581 1.57078', '0 1 0'), # Elbow 1 -> Elbow 2 (Pitch)
    'turntable_link_joint_dup_2': ('-0.000145 -0.000782 0.139796', '0.00167 0.01634 0.01070', '0 1 0'), # Elbow 2 -> Wrist (Pitch)
}

for j in root.findall('joint'):
    jname = j.get('name')
    if jname == 'fixed_base':
        continue
    
    if jname in clean_origins:
        xyz_val, rpy_val, axis_val = clean_origins[jname]
        orig = j.find('origin')
        if orig is not None:
            orig.set('xyz', xyz_val)
            orig.set('rpy', rpy_val)
        axis = j.find('axis')
        if axis is None:
            ET.SubElement(j, 'axis', {'xyz': axis_val})
        else:
            axis.set('xyz', axis_val)

    if j.get('type') in ('revolute', 'prismatic'):
        lim = j.find('limit')
        if lim is None:
            ET.SubElement(j, 'limit', {
                'lower':'-1.57','upper':'1.57','effort':'30.0','velocity':'2.0'})
        else:
            if float(lim.get('effort','0'))<=0:  lim.set('effort','30.0')
            if float(lim.get('velocity','0'))<=0: lim.set('velocity','2.0')

# Set visual/collision origins
for lk in root.findall('link'):
    ln = lk.get('name')
    if ln == 'world': continue

    vis = lk.find('visual')
    if vis is not None:
        vo = vis.find('origin')
        if vo is not None:
            if ln == 'turntable_link':
                vo.set('rpy', '-1.04042 -1.54565 -0.52117')
                vo.set('xyz', '0.00000 0.00000 0.00000')
            else:
                vo.set('rpy', '0.00000 0.00000 0.00000')
                vo.set('xyz', '0.00000 0.00000 0.00000')

    # Collision
    if lk.find('collision') is None and vis is not None:
        col = ET.SubElement(lk, 'collision', {'name': (vis.get('name') or ln) + '_col'})
        co_rpy = '-1.04042 -1.54565 -0.52117' if ln == 'turntable_link' else '0.00000 0.00000 0.00000'
        ET.SubElement(col, 'origin', {'xyz': '0.00000 0.00000 0.00000', 'rpy': co_rpy})
        vg = vis.find('geometry')
        if vg is not None:
            cg = ET.SubElement(col, 'geometry')
            vm = vg.find('mesh')
            if vm is not None: ET.SubElement(cg, 'mesh', vm.attrib)
    elif lk.find('collision') is not None:
        co = lk.find('collision/origin')
        if co is not None:
            co_rpy = '-1.04042 -1.54565 -0.52117' if ln == 'turntable_link' else '0.00000 0.00000 0.00000'
            co.set('rpy', co_rpy)
            co.set('xyz', '0.00000 0.00000 0.00000')

    # Inertial
    mass_map = {
        'basecase1_link':2.5, 'turntable_link':1.2, 'Elbow_1_link':1.0,
        'Elbow_2_link':0.8, 'Wrist_pitch._link':0.5
    }
    if lk.find('inertial') is None:
        mv = mass_map.get(ln, 0.1)
        ine = ET.SubElement(lk, 'inertial')
        ine_rpy = '-1.04042 -1.54565 -0.52117' if ln == 'turntable_link' else '0.00000 0.00000 0.00000'
        ET.SubElement(ine, 'origin', {'xyz':'0 0 0','rpy': ine_rpy})
        ET.SubElement(ine, 'mass', {'value': f'{mv:.3f}'})
        ET.SubElement(ine, 'inertia', {
            'ixx':'0.005','ixy':'0','ixz':'0',
            'iyy':'0.005','iyz':'0','izz':'0.005'})

def indent(e, lvl=0):
    i = "\n" + "  "*lvl
    if len(e):
        if not e.text or not e.text.strip(): e.text = i+"  "
        if not e.tail or not e.tail.strip(): e.tail = i
        for c in e: indent(c, lvl+1)
        if not c.tail or not c.tail.strip(): c.tail = i
    elif lvl and (not e.tail or not e.tail.strip()): e.tail = i

indent(root)
tree.write(urdf_out, encoding='utf-8', xml_declaration=True)
print(f'✅ {urdf_out} regenerated with flat turntable disk orientation!')
