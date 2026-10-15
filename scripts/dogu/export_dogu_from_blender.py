"""Export Dogu's per-link meshes and joint frames from the Blender model.

Run inside Blender (read-only: the .blend is never saved):

    blender --background --factory-startup cad/dogu/Dogu.blend \
            --python scripts/dogu/export_dogu_from_blender.py

Writes, relative to the repository root:
    urdf/dogu/meshes/dogu_<link>.stl   binary STL in the layout link frame, metres
    urdf/dogu/layout_model.json        links, joints (origin, axis), foot points,
                                       with joint zero = the .blend's parts layout

Then run `python3 scripts/dogu/build_dogu.py` to make the URDF and animations.

Why this exists: the Phobos export (cad/dogu/phobos_export/) has the right
joint tree but is 10x too big (the scene is in decimetres), only the body is
tagged as a visual, and the joint names are inconsistent.
"""
import json
import math
import os
import struct
import sys

import bmesh
import bpy
from mathutils import Matrix

SCENE_TO_M = 0.1          # the model was built 10x real size (2.23 "m" long body)
TARGET_TRIS = {"base": 9000, "default": 2500}   # decimate for the web twin

# Blender object names -> clean names (legs: fl/fr/bl/br; segments: hip/femur/tibia)
LINK_NAMES = {
    "Body_link": "base_link",
    "Hipfront-left_link": "fl_hip_link", "Femurfront-left_link": "fl_femur_link", "TibiaLfront-left_link": "fl_tibia_link",
    "Hipfront-right_link": "fr_hip_link", "Femurfront-right_link": "fr_femur_link", "TibiaLfront-right_link": "fr_tibia_link",
    "Hipback-left_link": "bl_hip_link", "Femurback-left_link": "bl_femur_link", "TibiaLback-left_link": "bl_tibia_link",
    "Hipback-right_link": "br_hip_link", "Femurback-right_link": "br_femur_link", "TibiaLback-right_link": "br_tibia_link",
}

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
OUT_DIR = os.path.join(ROOT, "urdf", "dogu")
MESH_DIR = os.path.join(OUT_DIR, "meshes")


def frame(ob):
    """World frame of an object without scale (link frames must be rigid)."""
    loc, rot, _scale = ob.matrix_world.decompose()
    return Matrix.LocRotScale(loc, rot, None)


def rpy(m):
    e = m.to_euler('XYZ')            # extrinsic XYZ == URDF roll-pitch-yaw
    return [round(e.x, 6), round(e.y, 6), round(e.z, 6)]


def write_stl(path, tris):
    with open(path, "wb") as f:
        f.write(b"Dogu link mesh (metres, link frame)".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
            vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
            nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
            n = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            f.write(struct.pack("<12fH", nx / n, ny / n, nz / n, *a, *b, *c, 0))


def link_mesh(link_ob):
    meshes = [c for c in link_ob.children if c.type == 'MESH']
    return meshes[0] if meshes else None


def main():
    os.makedirs(MESH_DIR, exist_ok=True)
    links = {o.name: o for o in bpy.data.objects if o.type == 'ARMATURE' and o.name in LINK_NAMES}
    missing = sorted(set(LINK_NAMES) - set(links))
    if missing:
        sys.exit(f"Dogu export: links not found in the .blend: {missing}")

    model = {"name": "dogu", "units": "m", "scale_from_blend": SCENE_TO_M,
             "source": "cad/dogu/Dogu.blend", "links": [], "joints": []}
    depsgraph = bpy.context.evaluated_depsgraph_get()

    for bname, ob in sorted(links.items(), key=lambda kv: LINK_NAMES[kv[0]]):
        name = LINK_NAMES[bname]
        entry = {"name": name, "blend_name": bname, "mesh": None, "foot": None}
        mesh_ob = link_mesh(ob)
        if mesh_ob is not None:
            tris_est = sum(max(0, len(p.vertices) - 2) for p in mesh_ob.data.polygons)
            target = TARGET_TRIS["base"] if name == "base_link" else TARGET_TRIS["default"]
            mod = mesh_ob.modifiers.new("dogu_export_decimate", 'DECIMATE')
            mod.ratio = min(1.0, target / max(1, tris_est))
            depsgraph.update()
            ev = mesh_ob.evaluated_get(depsgraph)
            me = ev.to_mesh()
            bm = bmesh.new()
            bm.from_mesh(me)
            bmesh.ops.triangulate(bm, faces=bm.faces[:])
            to_link = frame(ob).inverted() @ mesh_ob.matrix_world
            tris, far, far_d = [], None, -1.0
            for face in bm.faces:
                pts = []
                for v in face.verts:
                    p = to_link @ v.co
                    q = (p.x * SCENE_TO_M, p.y * SCENE_TO_M, p.z * SCENE_TO_M)
                    pts.append(q)
                    d = q[0] ** 2 + q[1] ** 2 + q[2] ** 2
                    if d > far_d:
                        far, far_d = q, d
                tris.append(pts)
            bm.free()
            ev.to_mesh_clear()
            mesh_ob.modifiers.remove(mod)
            fname = f"dogu_{name.replace('_link', '')}.stl"
            write_stl(os.path.join(MESH_DIR, fname), tris)
            entry["mesh"] = fname
            entry["triangles"] = len(tris)
            if name.endswith("tibia_link"):
                entry["foot"] = [round(c, 5) for c in far]   # tip of the tibia = foot contact
        model["links"].append(entry)

        if ob.parent is not None and ob.parent.name in links:
            rel = frame(ob.parent).inverted() @ frame(ob)
            t = rel.to_translation() * SCENE_TO_M
            model["joints"].append({
                "name": name.replace("_link", "_joint"),
                "blend_child": bname,
                "parent": LINK_NAMES[ob.parent.name],
                "child": name,
                "xyz": [round(t.x, 6), round(t.y, 6), round(t.z, 6)],
                "rpy": rpy(rel),
                "axis": [0.0, 0.0, 1.0],     # Phobos: each joint turns about its link's z
            })

    with open(os.path.join(OUT_DIR, "layout_model.json"), "w") as f:
        json.dump(model, f, indent=2)
    print(f"Dogu export: {len(model['links'])} links, {len(model['joints'])} joints -> {OUT_DIR}")


main()
