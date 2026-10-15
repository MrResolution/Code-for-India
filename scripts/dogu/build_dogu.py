#!/usr/bin/env python3
"""Build Dogu's URDF, runtime model and animations from the Blender export.

    python3 scripts/dogu/build_dogu.py

Input (from export_dogu_from_blender.py):
    urdf/dogu/layout_model.json   joint frames with zero = the .blend's parts layout
    urdf/dogu/meshes/*.stl        link meshes in those layout frames
Output:
    urdf/dogu/urdf/dogu.urdf      metric URDF, joint zero = standing
    urdf/dogu/model.json          the same kinematics for the web twin
    urdf/dogu/animations.json     poses + looping clips, radians from standing

The layout pose has the legs stretched flat, so the standing pose is solved
with inverse kinematics and then folded into the joint frames ("re-zeroing"):
every joint reads 0 when standing and animations are small offsets, the way a
servo's centre would be calibrated. Because every joint turns about its own z
axis, folding the angle into the joint origin (O' = O * Rz(stand)) leaves all
link frames, meshes and foot points exactly as they were.

Only the standard library is needed. Joint limits are provisional: Dogu has
no firmware or servo data yet.
"""
import argparse
import json
import math
import os

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DOGU = os.path.join(ROOT, "urdf", "dogu")
LEGS = ("fl", "fr", "bl", "br")
SEGMENTS = ("hip", "femur", "tibia")
LIMIT = math.radians(90)          # provisional: +-90 deg from standing, like a 180 deg servo
# Same absolute prefix as the other URDFs here; display_ros2.launch.py swaps it
# for wherever the repository actually lives.
DEFAULT_MESH_PREFIX = "file:///home/sabo/Documents/learn_/Hardware"


# ── 4x4 matrix helpers (row-major lists) ─────────────────────────────────
def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def origin(xyz, rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
            [-sp, cp * sr, cp * cr, xyz[2]],
            [0.0, 0.0, 0.0, 1.0]]


def to_xyz_rpy(m):
    """Inverse of origin(): URDF xyz + rpy (R = Rz(yaw) Ry(pitch) Rx(roll))."""
    r20 = max(-1.0, min(1.0, m[2][0]))
    if abs(r20) < 1.0 - 1e-9:
        pitch = math.asin(-r20)
        roll = math.atan2(m[2][1], m[2][2])
        yaw = math.atan2(m[1][0], m[0][0])
    else:                                   # gimbal lock: pitch = +-90 deg, put it all in roll
        yaw = 0.0
        pitch = math.pi / 2 if r20 < 0 else -math.pi / 2
        roll = math.atan2(m[0][1], m[0][2]) if r20 < 0 else math.atan2(-m[0][1], -m[0][2])
    xyz = [m[0][3], m[1][3], m[2][3]]
    back = origin(xyz, [roll, pitch, yaw])
    err = max(abs(back[i][j] - m[i][j]) for i in range(3) for j in range(3))
    if err > 1e-6:
        raise SystemExit(f"Dogu: rpy conversion failed (error {err:.1e})")
    return xyz, [roll, pitch, yaw]


def rot_z(q):
    c, s = math.cos(q), math.sin(q)
    return [[c, -s, 0.0, 0.0], [s, c, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]


def apply(m, p):
    return [m[i][0] * p[0] + m[i][1] * p[1] + m[i][2] * p[2] + m[i][3] for i in range(3)]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Leg:
    """base -> hip(q0) -> femur(q1) -> tibia(q2) -> foot point, for one leg."""

    def __init__(self, name, joints, links):
        self.name = name
        self.joint_names = [f"{name}_{seg}_joint" for seg in SEGMENTS]
        self.origins = [origin(joints[j]["xyz"], joints[j]["rpy"]) for j in self.joint_names]
        self.foot = links[f"{name}_tibia_link"]["foot"]

    def frames(self, q):
        """Base-frame transform of each joint frame (before its rotation)."""
        m = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
        out = []
        for o, qi in zip(self.origins, q):
            m = mat_mul(m, o)
            out.append(m)
            m = mat_mul(m, rot_z(qi))
        return out, m

    def points(self, q):
        frames, tip = self.frames(q)
        return [[f[0][3], f[1][3], f[2][3]] for f in frames] + [apply(tip, self.foot)]

    def axes(self, q):
        frames, _ = self.frames(q)
        return [[f[0][2], f[1][2], f[2][2]] for f in frames]

    def foot_at(self, q):
        return self.points(q)[3]

    def solve(self, target, q0, iters=80):
        """Damped least squares from q0, staying on the same turn as q0."""
        q = list(q0)
        for _ in range(iters):
            f = self.foot_at(q)
            err = [t - c for t, c in zip(target, f)]
            if math.sqrt(sum(e * e for e in err)) < 1e-5:
                break
            jac = []
            for k in range(3):
                dq = list(q)
                dq[k] += 1e-5
                jac.append([(a - b) / 1e-5 for a, b in zip(self.foot_at(dq), f)])
            jtj = [[sum(jac[a][i] * jac[b][i] for i in range(3)) + (1e-4 if a == b else 0.0)
                    for b in range(3)] for a in range(3)]
            jte = [sum(jac[a][i] * err[i] for i in range(3)) for a in range(3)]
            d = solve3(jtj, jte)
            step = max(abs(x) for x in d)
            if step > 0.3:
                d = [x * 0.3 / step for x in d]
            q = [qi + di for qi, di in zip(q, d)]
        return [s + wrap(x - s) for x, s in zip(q, q0)]


def solve3(a, b):
    """Solve a 3x3 linear system (Cramer's rule)."""
    def det(m):
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    d = det(a)
    if abs(d) < 1e-14:
        return [0.0, 0.0, 0.0]
    out = []
    for c in range(3):
        m = [row[:] for row in a]
        for r in range(3):
            m[r][c] = b[r]
        out.append(det(m) / d)
    return out


def stand_pose(leg, height):
    """Standing configuration in layout angles.

    The hip (abduction, about the body's long axis) is set so the femur and
    knee axes point sideways and the leg swings fore and aft. The femur/tibia
    pair then puts the foot `height` under the femur joint with the knee
    bent backwards (Spot style)."""
    hip = max((math.radians(d) for d in range(-180, 180)),
              key=lambda h: (abs(leg.axes([h, 0.0, 0.0])[1][1]), -abs(h)))
    femur_joint = leg.points([hip, 0.0, 0.0])[1]
    target = [femur_joint[0], femur_joint[1], femur_joint[2] - height]
    best, best_cost = None, 1e9
    for i in range(72):
        for k in range(72):
            q = [hip, wrap(math.radians(i * 5)), wrap(math.radians(k * 5))]
            knee, foot = leg.points(q)[2:]
            cost = sum((a - b) ** 2 for a, b in zip(foot, target))
            cost += 0.002 if knee[0] > foot[0] else 0.0     # knee behind the foot
            cost += 0.002 if knee[2] < foot[2] else 0.0     # and above it
            if cost < best_cost:
                best, best_cost = q, cost
    q = leg.solve(target, best)
    return q, target


def rezero(layout, stand):
    """Fold the standing angles into the joint origins (zero = standing).

    Old link frame at angle q:  O * Rz(q)
    New link frame at angle q': O * Rz(s) * Rz(q') = O * Rz(s + q')
    so q' = q - s and every link frame (and what is attached to it) is unchanged."""
    joints = {j["name"]: dict(j) for j in layout["joints"]}
    links = {l["name"]: dict(l) for l in layout["links"]}
    for j in joints.values():
        o = mat_mul(origin(j["xyz"], j["rpy"]), rot_z(stand[j["name"]]))
        j["xyz"], j["rpy"] = to_xyz_rpy(o)
    return joints, links


def build(args):
    with open(os.path.join(DOGU, "layout_model.json")) as f:
        layout = json.load(f)
    lj = {j["name"]: j for j in layout["joints"]}
    ll = {l["name"]: l for l in layout["links"]}
    layout_legs = {n: Leg(n, lj, ll) for n in LEGS}

    p = layout_legs["fl"].points([0.0, 0.0, 0.0])
    femur_len, tibia_len = math.dist(p[1], p[2]), math.dist(p[2], p[3])
    height = args.stand_ratio * (femur_len + tibia_len)

    stand, feet = {}, {}
    for n in LEGS:
        q, feet[n] = stand_pose(layout_legs[n], height)
        for jn, a in zip(layout_legs[n].joint_names, q):
            stand[jn] = a

    joints, links = rezero(layout, stand)
    legs = {n: Leg(n, joints, links) for n in LEGS}
    order = [j for n in LEGS for j in legs[n].joint_names]

    # Re-zeroed model at q = 0 must stand exactly where the layout model does at q = stand
    for n in LEGS:
        a = legs[n].foot_at([0.0, 0.0, 0.0])
        b = layout_legs[n].foot_at([stand[j] for j in layout_legs[n].joint_names])
        if math.dist(a, b) > 1e-6:
            raise SystemExit(f"Dogu: re-zeroing moved the {n} foot by {math.dist(a, b) * 1000:.3f} mm")

    residual = [0.0]
    zero = {n: [0.0, 0.0, 0.0] for n in LEGS}

    def pose_from(offsets, warm):
        """Joint angles (from standing) for feet moved by offsets[leg], metres."""
        out = {}
        for n in LEGS:
            dx, dy, dz = offsets.get(n, (0.0, 0.0, 0.0))
            tgt = [feet[n][0] + dx, feet[n][1] + dy, feet[n][2] + dz]
            out[n] = legs[n].solve(tgt, warm[n])
            residual[0] = max(residual[0], math.dist(legs[n].foot_at(out[n]), tgt))
        return out

    def flat(pose):
        return [round(a, 4) for n in LEGS for a in pose[n]]

    poses = {
        "stand": flat(zero),
        "crouch": flat(pose_from({n: (0.0, 0.0, 0.3 * height) for n in LEGS}, zero)),
        "sit": flat(pose_from({"fl": (0.0, 0.0, -0.04 * height), "fr": (0.0, 0.0, -0.04 * height),
                               "bl": (0.25 * height, 0.0, 0.45 * height),
                               "br": (0.25 * height, 0.0, 0.45 * height)}, zero)),
    }

    stride, lift = args.stride, args.lift

    def foot_path(phase):
        """Trot foot path: stance slides back on the ground, swing lifts it forward."""
        phase %= 1.0
        if phase < 0.5:
            return (stride * (0.5 - phase / 0.5), 0.0, 0.0)
        s = (phase - 0.5) / 0.5
        return (stride * (s - 0.5), 0.0, lift * math.sin(math.pi * s))

    phase_of = {"fl": 0.0, "br": 0.0, "fr": 0.5, "bl": 0.5}     # diagonal pairs
    clips = {}

    def clip(name, offsets_fn, duration):
        warm, seq = zero, []
        for i in range(args.frames):
            pose = pose_from(offsets_fn(i / args.frames), warm)
            warm = pose
            seq.append(flat(pose))
        clips[name] = {"loop": True, "duration": duration, "frames": seq}

    clip("walk", lambda t: {n: foot_path(t + phase_of[n]) for n in LEGS}, 1.0)

    def turn(t, direction):
        out = {}
        for n in LEGS:
            x, _, z = foot_path(t + phase_of[n])
            front = 1.0 if n in ("fl", "fr") else -1.0
            out[n] = (0.0, direction * front * x * 0.8, z)   # front/back feet sweep opposite ways
        return out
    clip("turn_left", lambda t: turn(t, 1.0), 1.0)
    clip("turn_right", lambda t: turn(t, -1.0), 1.0)

    def wave(t):
        a = 2.0 * math.pi * t
        rest = (0.0, 0.0, -0.04 * height)
        return {"fr": (0.35 * height, -0.08 * height, 0.75 * height + 0.12 * height * math.sin(2 * a)),
                "fl": rest, "bl": rest, "br": rest}
    clip("wave", wave, 1.6)
    clip("idle", lambda t: {n: (0.0, 0.0, 0.05 * height * math.sin(2 * math.pi * t)) for n in LEGS}, 3.0)

    for label, frames in [(k, [v]) for k, v in poses.items()] + [(k, c["frames"]) for k, c in clips.items()]:
        for fr in frames:
            for name, a in zip(order, fr):
                if abs(a) > LIMIT:
                    raise SystemExit(f"Dogu: {label}: {name} = {math.degrees(a):.0f} deg from standing "
                                     f"(limit +-{math.degrees(LIMIT):.0f}); make the motion smaller")
    if residual[0] > 0.002:
        raise SystemExit(f"Dogu: a keyframe misses its foot target by {residual[0] * 1000:.1f} mm "
                         "(out of reach?); reduce --stride/--lift or --stand-ratio")

    runtime = {
        "name": "dogu", "units": "m", "zero_pose": "standing",
        "note": "Generated by scripts/dogu/build_dogu.py; edit that script, not this file.",
        "links": [{"name": l["name"], "mesh": l["mesh"],
                   "foot": [round(c, 6) for c in l["foot"]] if l.get("foot") else None}
                  for l in sorted(links.values(), key=lambda l: l["name"])],
        "joints": [{"name": j["name"], "parent": j["parent"], "child": j["child"],
                    "xyz": [round(v, 6) for v in j["xyz"]], "rpy": [round(v, 6) for v in j["rpy"]],
                    "axis": [0.0, 0.0, 1.0], "limit": [-LIMIT, LIMIT]}
                   for j in sorted(joints.values(), key=lambda j: order.index(j["name"]))],
    }
    anims = {
        "robot": "dogu", "joints": order, "units": "rad (0 = standing)",
        "note": "Generated by scripts/dogu/build_dogu.py; edit that script, not this file.",
        "stand_height_m": round(height, 4), "max_foot_error_m": round(residual[0], 6),
        "poses": poses, "clips": clips,
    }
    with open(os.path.join(DOGU, "model.json"), "w") as f:
        json.dump(runtime, f, indent=1)
    with open(os.path.join(DOGU, "animations.json"), "w") as f:
        json.dump(anims, f, indent=1)
    write_urdf(runtime, args.mesh_prefix)
    print(f"Dogu: femur {femur_len * 1000:.1f} mm, tibia {tibia_len * 1000:.1f} mm, "
          f"feet {height * 1000:.1f} mm below the femur joints when standing")
    print(f"Dogu: {len(poses)} poses, {len(clips)} clips x {args.frames} frames, "
          f"max foot error {residual[0] * 1000:.3f} mm -> {DOGU}")


def write_urdf(model, mesh_prefix):
    colors = {"base": "0.24 0.55 0.85 1", "hip": "0.30 0.32 0.36 1",
              "femur": "0.93 0.62 0.20 1", "tibia": "0.85 0.86 0.88 1"}
    lines = ['<?xml version="1.0"?>',
             '<!-- Generated by scripts/dogu/build_dogu.py. Do not edit by hand. -->',
             '<!-- Joint zero = standing. Limits are provisional: Dogu has no servo data or firmware yet. -->',
             '<robot name="dogu">']
    for link in model["links"]:
        lines.append(f'  <link name="{link["name"]}">')
        if link["mesh"]:
            kind = next((k for k in colors if k in link["name"]), "base")
            lines += [
                '    <visual>',
                '      <origin xyz="0 0 0" rpy="0 0 0"/>',
                f'      <geometry><mesh filename="{mesh_prefix}/urdf/dogu/meshes/{link["mesh"]}"/></geometry>',
                f'      <material name="dogu_{kind}"><color rgba="{colors[kind]}"/></material>',
                '    </visual>',
            ]
        lines.append('  </link>')
    for j in model["joints"]:
        x, y, z = j["xyz"]
        r, p, yw = j["rpy"]
        lo, hi = j["limit"]
        lines += [
            f'  <joint name="{j["name"]}" type="revolute">',
            f'    <parent link="{j["parent"]}"/>',
            f'    <child link="{j["child"]}"/>',
            f'    <origin xyz="{x:.6f} {y:.6f} {z:.6f}" rpy="{r:.6f} {p:.6f} {yw:.6f}"/>',
            '    <axis xyz="0 0 1"/>',
            f'    <limit lower="{lo:.5f}" upper="{hi:.5f}" effort="0.25" velocity="6.0"/>',
            '  </joint>',
        ]
    lines.append('</robot>')
    os.makedirs(os.path.join(DOGU, "urdf"), exist_ok=True)
    with open(os.path.join(DOGU, "urdf", "dogu.urdf"), "w") as f:
        f.write("\n".join(lines) + "\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--mesh-prefix", default=DEFAULT_MESH_PREFIX,
                    help="URL prefix for mesh files in the URDF (default matches the other URDFs)")
    ap.add_argument("--stand-ratio", type=float, default=0.8,
                    help="standing foot depth below the femur joint, as a fraction of femur+tibia")
    ap.add_argument("--stride", type=float, default=0.03, help="walk stride (m)")
    ap.add_argument("--lift", type=float, default=0.018, help="walk foot lift (m)")
    ap.add_argument("--frames", type=int, default=24, help="keyframes per clip")
    build(ap.parse_args())


if __name__ == "__main__":
    main()
