import xml.etree.ElementTree as ET
import os

urdf_in = '/home/sabo/Documents/learn_/Hardware/urdf/unnamed/urdf/unnamed_gazebo.urdf'
urdf_out = '/home/sabo/Documents/learn_/Hardware/urdf/unnamed/urdf/unnamed_scaled.urdf'

tree = ET.parse(urdf_in)
root = tree.getroot()

SCALE_FACTOR = 0.1  # Joint origins were exported in decimeters (10 cm units)

for joint in root.findall('joint'):
    if joint.get('name') == 'fixed_base':
        continue
    origin = joint.find('origin')
    if origin is not None and 'xyz' in origin.attrib:
        xyz_str = origin.get('xyz')
        coords = [float(c) * SCALE_FACTOR for c in xyz_str.split()]
        origin.set('xyz', f"{coords[0]:.6f} {coords[1]:.6f} {coords[2]:.6f}")

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
print(f"Scaled joint origins by {SCALE_FACTOR} -> written to {urdf_out}")
