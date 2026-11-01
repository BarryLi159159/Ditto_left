"""Mirror a URDF across a link-frame plane to make a left-hand model.

Every link frame F is replaced by its reflection M F M (M = diag with -1 on the
mirror axis). For each joint/visual/collision/inertial origin (R, p):
    p' = M p,   R' = M R M
Revolute axes are pseudo-vectors, so a' = -M a; with that, the same joint
angle q produces the mirror-image pose and joint limits stay unchanged.
Meshes are reflected vertex-wise (M v) with face winding flipped.

Usage: python mirror_urdf.py <src.urdf> <dst.urdf> [--axis y] [--drop middle]
"""
import argparse
import copy
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def R_to_rpy(R):
    sp = -R[2, 0]
    if abs(sp) < 1 - 1e-9:
        p = np.arcsin(sp)
        r = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(R[1, 0], R[0, 0])
    else:  # gimbal lock: fold yaw into roll
        p = np.pi / 2 * np.sign(sp)
        r = np.arctan2(-R[1, 2], R[1, 1])
        y = 0.0
    return r, p, y


def fmt(v):
    return " ".join(f"{x:.12g}" for x in v)


def mirror_origin(el, M):
    if el is None:
        return
    p = np.array([float(x) for x in el.get("xyz", "0 0 0").split()])
    rpy = [float(x) for x in el.get("rpy", "0 0 0").split()]
    R = rpy_to_R(*rpy)
    el.set("xyz", fmt(M @ p))
    el.set("rpy", fmt(R_to_rpy(M @ R @ M)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--axis", default="y", choices="xyz")
    ap.add_argument("--drop", action="append", default=[], help="drop links/joints whose name contains this")
    ap.add_argument("--name", default=None)
    a = ap.parse_args()

    src, dst = Path(a.src).resolve(), Path(a.dst).resolve()
    M = np.eye(3)
    M["xyz".index(a.axis), "xyz".index(a.axis)] = -1.0

    tree = ET.parse(src)
    root = tree.getroot()
    if a.name:
        root.set("name", a.name)

    def dropped(name):
        return any(d in (name or "") for d in a.drop)

    for el in list(root):
        if el.tag in ("link", "joint") and dropped(el.get("name")):
            root.remove(el)
        elif el.tag == "joint" and (dropped(el.find("parent").get("link")) or dropped(el.find("child").get("link"))):
            root.remove(el)

    for j in root.findall("joint"):
        mirror_origin(j.find("origin"), M)
        ax = j.find("axis")
        if ax is not None:
            v = np.array([float(x) for x in ax.get("xyz").split()])
            ax.set("xyz", fmt(-(M @ v) + 0.0))

    dst.parent.mkdir(parents=True, exist_ok=True)
    (dst.parent / "meshes").mkdir(exist_ok=True)
    done = set()
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is not None:
            mirror_origin(inertial.find("origin"), M)
            I = inertial.find("inertia")
            if I is not None:
                T = np.array([[float(I.get(k, 0)) for k in row] for row in
                              (("ixx", "ixy", "ixz"), ("ixy", "iyy", "iyz"), ("ixz", "iyz", "izz"))])
                T = M @ T @ M
                for k, (i, jj) in {"ixx": (0, 0), "ixy": (0, 1), "ixz": (0, 2), "iyy": (1, 1), "iyz": (1, 2), "izz": (2, 2)}.items():
                    I.set(k, f"{T[i, jj]:.12g}")
        for tag in ("visual", "collision"):
            for vc in link.findall(tag):
                mirror_origin(vc.find("origin"), M)
                mesh = vc.find("geometry/mesh")
                if mesh is None:
                    continue
                rel = mesh.get("filename")
                if rel.startswith("package://"):
                    raise SystemExit(f"package:// mesh paths not supported: {rel}")
                if rel not in done:
                    m = trimesh.load(src.parent / rel, force="mesh")
                    H = np.eye(4)
                    H[:3, :3] = M
                    m.apply_transform(H)  # trimesh flips winding for a reflection
                    m.export(dst.parent / rel)
                    done.add(rel)

    ET.indent(tree, space="  ")
    tree.write(dst, xml_declaration=True, encoding="utf-8")
    print(f"wrote {dst} ({len(root.findall('joint'))} joints, {len(done)} meshes)")


if __name__ == "__main__":
    main()
