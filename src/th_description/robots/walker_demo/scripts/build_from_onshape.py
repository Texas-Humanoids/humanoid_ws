"""Build the training model for the demo walker from the raw Onshape export (pkg_002).

    python build_from_onshape.py <pkg_002 dir> [--out <walker_demo dir>]

Reads   <pkg_002>/urdf/pkg_002.urdf + <pkg_002>/meshes/*.stl
Writes  <out>/urdf/walker_demo.urdf, <out>/mjcf/walker_demo.xml, <out>/mjcf/scene.xml,
        <out>/meshes/<body>.stl, <out>/build_report.json

What it changes, and why (each one is a real problem in the raw export):
  1. 170 links / 159 fixed joints are merged into 11 rigid bodies (exact mass, COM and inertia
     composition). The Onshape tree is one link per part, which nothing downstream needs.
  2. Base frame: x forward, y left, z up (REP-103), origin at the pelvis centre on the hip-pitch
     axis. The raw export has the robot facing -y with the origin off to one side.
  3. Joint conventions made identical on both legs, matching WalkerV2 and the deploy stack:
       pitch about +y (hip pitch +: foot moves back; knee +: flexes; ankle +: toes down),
       roll about +x (+: foot moves to +y, i.e. left), yaw about +z.
     The raw export has mirrored axes on the right leg (left knee about -x_raw, right about +x_raw),
     so the same joint angle meant different motions per side.
  4. Collision geometry (the raw export has none): one box per body from its mesh bounds, plus a
     flat 2 cm sole box under each foot (that's what touches the ground in training).
  5. Effort / velocity limits from the RobStride datasheet peaks and our 36 V measurements,
     replacing the placeholder 1 Nm / 1 rad/s on every joint.
  6. Measured armature and Coulomb friction on each joint (sysid 2026-11-15) in the MJCF.
  7. Optional: SYMMETRIZE copies the left-right average masses onto both sides (the export has
     the left RS06 at 0.745 kg and the right at 0.630; datasheet is 0.621).

Joint limits are taken from the URDF as-is (after the sign flip), except where LIMIT_OVERRIDE says.
They are CAD mate limits: verify them on the robot with tools/walker/bringup/jog.py (limit finder).
"""
from __future__ import annotations

import argparse
import json
import struct
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import numpy as np

# --------------------------------------------------------------------------------------------
# Hardware numbers (one place). Joint-side, from RobStride datasheets + our sysid at 36 V.
# --------------------------------------------------------------------------------------------
MOTOR_OF = {"hip_pitch": "RS03", "hip_roll": "RS03", "hip_yaw": "RS02", "knee_pitch": "RS03", "ankle_pitch": "RS06"}
PEAK_TORQUE = {"RS03": 60.0, "RS02": 17.0, "RS06": 36.0}            # Nm, datasheet peak
SPEED_36V = {"RS03": 16.4, "RS02": 33.3, "RS06": 37.7}             # rad/s, measured top speed scaled to 36 V
ARMATURE = {"RS03": 0.07, "RS02": 0.013, "RS06": 0.018}             # kg m^2, sysid Bode/time fit
FRICTION = {"RS03": 0.40, "RS02": 0.12, "RS06": 0.18}               # Nm Coulomb, sysid
DAMPING = {"RS03": 0.013, "RS02": 0.0009, "RS06": 0.0018}           # Nm s/rad viscous, sysid

JOINTS = ["hip_pitch", "hip_roll", "hip_yaw", "knee_pitch", "ankle_pitch"]
CANON_AXIS = {"hip_pitch": (0, 1, 0), "hip_roll": (1, 0, 0), "hip_yaw": (0, 0, 1),
              "knee_pitch": (0, 1, 0), "ankle_pitch": (0, 1, 0)}
LINK_OF = {"hip_pitch": "hip_pitch_link", "hip_roll": "hip_roll_link", "hip_yaw": "thigh_link",
           "knee_pitch": "shank_link", "ankle_pitch": "foot_link"}
# Clean-convention limits to use instead of the URDF's, e.g. {"left_hip_roll_joint": (-0.35, 0.785)}.
LIMIT_OVERRIDE: dict[str, tuple[float, float]] = {}
SYMMETRIZE = True
SOLE_THICKNESS = 0.02


def rpy2R(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def vec(s):
    return np.array([float(v) for v in s.split()])


def read_stl(path: Path) -> np.ndarray:
    """Triangles [N, 3, 3]. Binary or ASCII."""
    data = path.read_bytes()
    if len(data) >= 84:
        n = struct.unpack("<I", data[80:84])[0]
        if 84 + 50 * n == len(data):
            rec = np.frombuffer(data[84:], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
            return rec["v"].astype(np.float64)
    pts = [list(map(float, l.split()[1:4])) for l in data.decode(errors="ignore").splitlines() if l.strip().startswith("vertex")]
    return np.array(pts).reshape(-1, 3, 3)


def write_stl(path: Path, tris: np.ndarray) -> None:
    tris = tris.astype(np.float32)
    nrm = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12
    rec = np.zeros(len(tris), dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    rec["n"], rec["v"] = nrm, tris
    with open(path, "wb") as f:
        f.write(b"walker_demo merged mesh".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        f.write(rec.tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pkg", type=Path)
    ap.add_argument("--out", type=Path, default=Path(__file__).resolve().parents[1])
    args = ap.parse_args()
    src = args.pkg / "urdf" / "pkg_002.urdf"
    out = args.out
    for d in ("urdf", "mjcf", "meshes"):
        (out / d).mkdir(parents=True, exist_ok=True)
    report: dict = {"source": str(src), "warnings": []}

    root = ET.parse(src).getroot()
    links = {l.get("name"): l for l in root.findall("link")}
    jof, kids = {}, defaultdict(list)
    for j in root.findall("joint"):
        p, c = j.find("parent").get("link"), j.find("child").get("link")
        jof[c] = j
        kids[p].append(c)
    rootlink = [n for n in links if n not in jof][0]

    # ---- forward kinematics at q = 0 (raw frame)
    W = {rootlink: np.eye(4)}
    order = [rootlink]
    i = 0
    while i < len(order):
        p = order[i]; i += 1
        for c in kids[p]:
            o = jof[c].find("origin")
            T = np.eye(4)
            T[:3, :3] = rpy2R(vec(o.get("rpy"))); T[:3, 3] = vec(o.get("xyz"))
            W[c] = W[p] @ T
            order.append(c)

    # ---- which body each link belongs to
    raw_joint_of_body = {}
    body_of = {rootlink: "base"}
    for n in order[1:]:
        j = jof[n]
        body_of[n] = j.get("name") if j.get("type") != "fixed" else body_of[jof[n].find("parent").get("link")]
        if j.get("type") != "fixed":
            raw_joint_of_body[j.get("name")] = j
    expected = {f"{k}_{s}" for k in JOINTS for s in "lr"}
    assert set(raw_joint_of_body) == expected, f"unexpected revolute joints: {sorted(raw_joint_of_body)}"

    def clean_name(raw):  # hip_pitch_l -> left_hip_pitch_joint
        k, s = raw[:-2], raw[-1]
        return f"{'left' if s == 'l' else 'right'}_{k}_joint"

    def link_name(raw):
        k, s = raw[:-2], raw[-1]
        return f"{'left' if s == 'l' else 'right'}_{LINK_OF[k]}"

    # ---- new world frame: raw -y (front) -> +x, raw +x (robot left) -> +y
    jpos_raw = {r: W[raw_joint_of_body[r].find("child").get("link")][:3, 3] for r in raw_joint_of_body}
    C = (jpos_raw["hip_pitch_l"] + jpos_raw["hip_pitch_r"]) / 2
    Rz = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # +90 deg about z
    to_new = lambda p: Rz @ (p - C)
    assert to_new(jpos_raw["hip_roll_l"])[1] > 0, "left leg did not end up on +y; check the frame"

    # ---- joints in the clean convention
    joints = {}
    for r, j in raw_joint_of_body.items():
        k = r[:-2]
        child = j.find("child").get("link")
        ax_new = Rz @ (W[child][:3, :3] @ vec(j.find("axis").get("xyz")))
        canon = np.array(CANON_AXIS[k], float)
        d = float(ax_new @ canon)
        if abs(abs(d) - 1) > 1e-6:
            report["warnings"].append(f"{r}: axis {ax_new.round(4)} is not aligned with {canon}")
        sgn = 1.0 if d > 0 else -1.0
        lim = j.find("limit")
        lo, hi = sorted((sgn * float(lim.get("lower")), sgn * float(lim.get("upper"))))
        name = clean_name(r)
        urdf_lim = (lo, hi)
        if name in LIMIT_OVERRIDE:
            lo, hi = LIMIT_OVERRIDE[name]
        mot = MOTOR_OF[k]
        joints[name] = dict(raw=r, kind=k, sign_vs_raw=sgn, pos=to_new(jpos_raw[r]), axis=canon,
                            lower=lo, upper=hi, urdf_limits=urdf_lim, motor=mot,
                            effort=PEAK_TORQUE[mot], velocity=SPEED_36V[mot],
                            armature=ARMATURE[mot], friction=FRICTION[mot], damping=DAMPING[mot])
    report["joints"] = {n: {kk: (v.tolist() if isinstance(v, np.ndarray) else v) for kk, v in d.items()} for n, d in joints.items()}

    # mirror check: left/right joint positions at q=0 should mirror about y=0
    for k in JOINTS:
        pl, pr = joints[f"left_{k}_joint"]["pos"], joints[f"right_{k}_joint"]["pos"]
        err = np.abs(pl - pr * np.array([1, -1, 1])).max()
        if err > 1e-4:
            report["warnings"].append(f"{k}: left/right joint positions don't mirror (err {err * 1000:.2f} mm)")
        ll, lr = (joints[f"left_{k}_joint"]["urdf_limits"], joints[f"right_{k}_joint"]["urdf_limits"])
        mirrored = (-lr[1], -lr[0]) if k in ("hip_roll", "hip_yaw") else lr
        if np.abs(np.array(ll) - np.array(mirrored)).max() > 1e-3:
            report["warnings"].append(f"{k}: URDF limits are not left/right symmetric: left {ll}, right {lr}")

    # ---- bodies: name, parent, origin (new coords, world-aligned at q=0)
    bodies = {"base": dict(name="base_link", parent=None, origin=np.zeros(3), joint=None)}
    for r, j in raw_joint_of_body.items():
        parent_body = body_of[j.find("parent").get("link")]
        bodies[r] = dict(name=link_name(r), parent=parent_body, origin=joints[clean_name(r)]["pos"], joint=clean_name(r))

    # ---- mass properties per body, and meshes
    members = defaultdict(list)
    for n, b in body_of.items():
        members[b].append(n)
    zero_mass = []
    tri_by_body = {}
    for b, ns in members.items():
        m_tot, mc, inertia_terms = 0.0, np.zeros(3), []
        tris = []
        for n in ns:
            L = links[n]
            Rl = Rz @ W[n][:3, :3]
            pl = to_new(W[n][:3, 3]) - bodies[b]["origin"]
            inn = L.find("inertial")
            if inn is not None:
                m = float(inn.find("mass").get("value"))
                if m < 1e-6 and L.find("visual") is not None:
                    zero_mass.append(n)
                o = inn.find("origin")
                Ri = rpy2R(vec(o.get("rpy"))) if o is not None else np.eye(3)
                ci = vec(o.get("xyz")) if o is not None else np.zeros(3)
                I = inn.find("inertia")
                Il = np.array([[float(I.get("ixx")), float(I.get("ixy")), float(I.get("ixz"))],
                               [float(I.get("ixy")), float(I.get("iyy")), float(I.get("iyz"))],
                               [float(I.get("ixz")), float(I.get("iyz")), float(I.get("izz"))]])
                Rw = Rl @ Ri
                com = pl + Rl @ ci
                m_tot += m; mc += m * com
                inertia_terms.append((m, com, Rw @ Il @ Rw.T, n))
            for v in L.findall("visual"):
                fn = v.find("geometry/mesh").get("filename").split("/")[-1]
                t = read_stl(args.pkg / "meshes" / fn)
                vo = v.find("origin")
                Rv = rpy2R(vec(vo.get("rpy"))) if vo is not None else np.eye(3)
                pv = vec(vo.get("xyz")) if vo is not None else np.zeros(3)
                tris.append(((t @ Rv.T + pv) @ Rl.T) + pl)
        com = mc / m_tot
        Ic = np.zeros((3, 3))
        for m, c, Iw, _ in inertia_terms:
            d = c - com
            Ic += Iw + m * (d @ d * np.eye(3) - np.outer(d, d))
        bodies[b].update(mass=m_tot, com=com, inertia=Ic, members=[t[3] for t in inertia_terms])
        tri_by_body[b] = np.concatenate(tris) if tris else np.zeros((0, 3, 3))
    report["zero_mass_parts"] = zero_mass
    if zero_mass:
        report["warnings"].append(f"{len(zero_mass)} visual parts have zero mass (no material in Onshape): {zero_mass}")

    # ---- symmetrize leg masses (left/right average, mirrored)
    report["mass_raw"] = {bodies[b]["name"]: round(bodies[b]["mass"], 4) for b in bodies}
    if SYMMETRIZE:
        M = np.diag([1, -1, 1])
        for k in JOINTS:
            bl, br = bodies[f"{k}_l"], bodies[f"{k}_r"]
            if abs(bl["mass"] - br["mass"]) / max(bl["mass"], br["mass"]) > 0.02:
                report["warnings"].append(
                    f"{k}: left {bl['mass']:.3f} kg vs right {br['mass']:.3f} kg in CAD; averaged (SYMMETRIZE)")
            m = (bl["mass"] + br["mass"]) / 2
            com = (bl["com"] + M @ br["com"]) / 2
            I = (bl["inertia"] + M @ br["inertia"] @ M) / 2
            bl.update(mass=m, com=com, inertia=I)
            br.update(mass=m, com=M @ com, inertia=M @ I @ M)

    # ---- collision boxes from mesh bounds (body frame), sole box under each foot
    for b, B in bodies.items():
        V = tri_by_body[b].reshape(-1, 3)
        lo, hi = V.min(0), V.max(0)
        B["bbox"] = (lo, hi)
        if B["name"].endswith("foot_link"):
            # Sole: full footprint, 2 cm thick, bottom face at the lowest mesh point.
            B["box_center"] = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2] + SOLE_THICKNESS / 2])
            B["box_half"] = np.array([(hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2, SOLE_THICKNESS / 2])
            B["sole_z"] = lo[2]
            # flatness: lowest point at the toe vs at the heel
            near = V[V[:, 2] < lo[2] + 0.002]
            B["sole_contact_x"] = (near[:, 0].min(), near[:, 0].max())
        else:
            B["box_center"] = (lo + hi) / 2
            B["box_half"] = (hi - lo) / 2

    # ---- meshes
    for b, B in bodies.items():
        write_stl(out / "meshes" / f"{B['name']}.stl", tri_by_body[b].astype(np.float32))

    # ---- standing geometry
    sole_z = bodies["ankle_pitch_l"]["origin"][2] + bodies["ankle_pitch_l"]["sole_z"]
    total_mass = sum(B["mass"] for B in bodies.values())
    com_all = sum(B["mass"] * (B["origin"] + B["com"]) for B in bodies.values()) / total_mass
    report.update(
        total_mass=round(total_mass, 4), com_at_zero=com_all.round(4).tolist(),
        base_height_straight_legs=round(-sole_z, 4),
        bodies={B["name"]: dict(mass=round(B["mass"], 4), com=B["com"].round(4).tolist(),
                                box_center=B["box_center"].round(4).tolist(), box_size=(2 * B["box_half"]).round(4).tolist(),
                                n_tris=int(len(tri_by_body[b])), parts=len(members[b]))
                for b, B in bodies.items()},
        foot_sole_contact_x=[round(float(v), 4) for v in bodies["ankle_pitch_l"]["sole_contact_x"]],
    )

    # ---- emit URDF
    def fmt(a):
        return " ".join(f"{float(x):.6g}" for x in a)

    U = ['<?xml version="1.0"?>', '<!-- GENERATED by scripts/build_from_onshape.py from pkg_002.urdf. Do not edit. -->',
         '<robot name="walker_demo">']
    for b, B in bodies.items():
        I = B["inertia"]
        U.append(f'  <link name="{B["name"]}">')
        U.append(f'    <inertial><origin xyz="{fmt(B["com"])}" rpy="0 0 0"/><mass value="{B["mass"]:.6g}"/>'
                 f'<inertia ixx="{I[0,0]:.6e}" ixy="{I[0,1]:.6e}" ixz="{I[0,2]:.6e}" iyy="{I[1,1]:.6e}" iyz="{I[1,2]:.6e}" izz="{I[2,2]:.6e}"/></inertial>')
        U.append(f'    <visual><geometry><mesh filename="package://walker_demo/meshes/{B["name"]}.stl"/></geometry></visual>')
        U.append(f'    <collision name="{B["name"]}_box"><origin xyz="{fmt(B["box_center"])}" rpy="0 0 0"/>'
                 f'<geometry><box size="{fmt(2 * B["box_half"])}"/></geometry></collision>')
        U.append("  </link>")
    U.append('  <link name="imu_link"/>')
    for b, B in bodies.items():
        if B["joint"] is None:
            continue
        J = joints[B["joint"]]
        par = bodies[B["parent"]]
        U.append(f'  <joint name="{B["joint"]}" type="revolute">')
        U.append(f'    <origin xyz="{fmt(B["origin"] - par["origin"])}" rpy="0 0 0"/><parent link="{par["name"]}"/><child link="{B["name"]}"/>')
        U.append(f'    <axis xyz="{fmt(J["axis"])}"/>')
        U.append(f'    <limit lower="{J["lower"]:.4f}" upper="{J["upper"]:.4f}" effort="{J["effort"]}" velocity="{J["velocity"]}"/>'
                 f'<dynamics damping="{J["damping"]}" friction="{J["friction"]}"/>  <!-- {J["motor"]} -->')
        U.append("  </joint>")
    U.append('  <joint name="imu_joint" type="fixed"><origin xyz="0 0 0" rpy="0 0 0"/><parent link="base_link"/><child link="imu_link"/></joint>')
    U.append("</robot>")
    (out / "urdf" / "walker_demo.urdf").write_text("\n".join(U) + "\n")

    # ---- emit MJCF (same names and structure as walker_v2.xml, so the training code carries over)
    def body_xml(b, depth):
        B = bodies[b]
        ind = "  " * depth
        rel = B["origin"] - (bodies[B["parent"]]["origin"] if B["parent"] else np.zeros(3))
        pos = rel if B["parent"] else np.array([0, 0, 0.70])
        X = [f'{ind}<body name="{B["name"]}" pos="{fmt(pos)}">']
        I = B["inertia"]
        X.append(f'{ind}  <inertial pos="{fmt(B["com"])}" mass="{B["mass"]:.6g}" '
                 f'fullinertia="{I[0,0]:.6e} {I[1,1]:.6e} {I[2,2]:.6e} {I[0,1]:.6e} {I[0,2]:.6e} {I[1,2]:.6e}"/>')
        if B["joint"] is None:
            X.append(f'{ind}  <freejoint name="floating_base_joint"/>')
        else:
            J = joints[B["joint"]]
            X.append(f'{ind}  <joint name="{B["joint"]}" axis="{fmt(J["axis"])}" range="{J["lower"]:.4f} {J["upper"]:.4f}" '
                     f'actuatorfrcrange="-{J["effort"]} {J["effort"]}" armature="{J["armature"]}" '
                     f'frictionloss="{J["friction"]}" damping="{J["damping"]}"/>')
        X.append(f'{ind}  <geom type="mesh" mesh="{B["name"]}" class="visual"/>')
        gname = B["name"].replace("_link", "") + "_collision" if B["name"].endswith("foot_link") else B["name"] + "_collision"
        X.append(f'{ind}  <geom name="{gname}" type="box" size="{fmt(B["box_half"])}" pos="{fmt(B["box_center"])}" class="collision"/>')
        if B["name"].endswith("foot_link"):
            side = B["name"].split("_")[0]
            c = B["box_center"]
            X.append(f'{ind}  <site name="{side}_foot" pos="{fmt([c[0], c[1], B["sole_z"]])}" size="0.01"/>')
        if B["joint"] is None:
            X.append(f'{ind}  <site name="imu" pos="0 0 0" size="0.01"/>')
        for c in [k for k, v in bodies.items() if v["parent"] == b]:
            X += body_xml(c, depth + 1)
        X.append(f"{ind}</body>")
        return X

    X = ['<mujoco model="walker_demo">',
         '  <!-- GENERATED by scripts/build_from_onshape.py from pkg_002.urdf. Do not edit. -->',
         '  <compiler angle="radian" meshdir="../meshes/"/>',
         '  <default>',
         '    <default class="visual"><geom contype="0" conaffinity="0" group="2" mass="0" density="0" rgba="0.3 0.3 0.33 1"/></default>',
         '    <default class="collision"><geom group="3" mass="0" density="0" rgba="0.2 0.6 0.2 0.4"/></default>',
         '  </default>',
         '  <asset>'] + [f'    <mesh name="{B["name"]}" file="{B["name"]}.stl"/>' for B in bodies.values()] + ['  </asset>', '  <worldbody>']
    X += body_xml("base", 2)
    X += ['  </worldbody>', '  <sensor>',
          '    <gyro site="imu" name="imu_ang_vel"/>', '    <velocimeter site="imu" name="imu_lin_vel"/>',
          '    <accelerometer site="imu" name="imu_lin_acc"/>',
          '    <framezaxis objtype="body" objname="world" reftype="site" refname="imu" name="imu_upvector"/>',
          '    <subtreeangmom body="base_link" name="root_angmom"/>', '  </sensor>', '</mujoco>']
    (out / "mjcf" / "walker_demo.xml").write_text("\n".join(X) + "\n")
    (out / "mjcf" / "scene.xml").write_text("""<mujoco model="walker_demo_scene">
  <include file="walker_demo.xml"/>
  <option timestep="0.001"/>
  <statistic center="0 0 0.4" extent="1.2"/>
  <visual><headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/><global azimuth="135" elevation="-15"/></visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
  </worldbody>
  <equality><weld name="gantry" body1="base_link" active="false"/></equality>
</mujoco>
""")
    (out / "build_report.json").write_text(json.dumps(report, indent=1, default=float))
    print(json.dumps({k: report[k] for k in ("total_mass", "com_at_zero", "base_height_straight_legs", "foot_sole_contact_x")}, indent=1))
    for w in report["warnings"]:
        print("WARN", w)
    for n, J in joints.items():
        print(f"{n:26s} sign_vs_raw {J['sign_vs_raw']:+.0f}  limits [{J['lower']:+.3f}, {J['upper']:+.3f}]  pos {J['pos'].round(4)}")
    for B in bodies.values():
        print(f"{B['name']:24s} {B['mass']:.3f} kg  box {(2*B['box_half']).round(3)} @ {B['box_center'].round(3)}")


if __name__ == "__main__":
    main()
