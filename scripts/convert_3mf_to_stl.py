#!/usr/bin/env python3
"""
3MF to Binary STL Converter
---------------------------
Extracts all individual 3D objects/meshes embedded inside a 3MF archive file
and converts each one into a standalone binary .STL file.

Usage:
  python3 scripts/convert_3mf_to_stl.py path/to/file.3mf [output_directory]
"""

import sys
import os
import math
import struct
import zipfile
import xml.etree.ElementTree as ET

def calc_normal(v1, v2, v3):
    ax, ay, az = v2[0]-v1[0], v2[1]-v1[1], v2[2]-v1[2]
    bx, by, bz = v3[0]-v1[0], v3[1]-v1[1], v3[2]-v1[2]
    nx, ny, nz = ay*bz - az*by, az*bx - ax*bz, ax*by - ay*bx
    l = math.sqrt(nx*nx + ny*ny + nz*nz)
    return (nx/l, ny/l, nz/l) if l > 0 else (0.0, 0.0, 0.0)

def convert_3mf(mf_path, out_dir):
    if not os.path.exists(mf_path):
        print(f"❌ Error: File '{mf_path}' does not exist.")
        return

    os.makedirs(out_dir, exist_ok=True)
    base_prefix = os.path.splitext(os.path.basename(mf_path))[0]
    extracted_count = 0

    with zipfile.ZipFile(mf_path, 'r') as z:
        for name in z.namelist():
            if name.endswith('.model'):
                xml_content = z.read(name)
                root = ET.fromstring(xml_content)
                ns = {'3mf': 'http://schemas.microsoft.com/3dmanufacturing/core/2015/02'}
                
                for obj in root.iter('{http://schemas.microsoft.com/3dmanufacturing/core/2015/02}object'):
                    obj_id = obj.attrib.get('id', 'part')
                    obj_name = obj.attrib.get('name', f'part_{obj_id}')
                    
                    mesh = obj.find('3mf:mesh', ns)
                    if mesh is not None:
                        vertices = []
                        verts_elem = mesh.find('3mf:vertices', ns)
                        if verts_elem is not None:
                            for v in verts_elem.findall('3mf:vertex', ns):
                                vertices.append((float(v.attrib['x']), float(v.attrib['y']), float(v.attrib['z'])))
                        
                        triangles = []
                        tris_elem = mesh.find('3mf:triangles', ns)
                        if tris_elem is not None:
                            for t in tris_elem.findall('3mf:triangle', ns):
                                triangles.append((int(t.attrib['v1']), int(t.attrib['v2']), int(t.attrib['v3'])))
                        
                        if vertices and triangles:
                            safe_name = ''.join(c if c.isalnum() or c in ('_', '-') else '_' for c in obj_name)
                            stl_filename = os.path.join(out_dir, f'{base_prefix}_{safe_name}_obj{obj_id}.stl')
                            
                            with open(stl_filename, 'wb') as f:
                                header = f'Extracted from {os.path.basename(mf_path)} - Object {obj_id}'.ljust(80).encode('utf-8')[:80]
                                f.write(header)
                                f.write(struct.pack('<I', len(triangles)))
                                for v1_i, v2_i, v3_i in triangles:
                                    v1, v2, v3 = vertices[v1_i], vertices[v2_i], vertices[v3_i]
                                    norm = calc_normal(v1, v2, v3)
                                    f.write(struct.pack('<3f', *norm))
                                    f.write(struct.pack('<3f', *v1))
                                    f.write(struct.pack('<3f', *v2))
                                    f.write(struct.pack('<3f', *v3))
                                    f.write(struct.pack('<H', 0))
                            
                            extracted_count += 1
                            print(f"  ✅ Extracted: {os.path.basename(stl_filename)} ({len(triangles)} triangles)")

    print(f"\n🎉 Successfully extracted {extracted_count} STL files into '{out_dir}'!")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 scripts/convert_3mf_to_stl.py <input.3mf> [output_directory]")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else "extracted_stls"
    convert_3mf(input_file, output_dir)
