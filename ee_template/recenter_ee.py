#!/usr/bin/env python3
"""Re-center an end-effector USD so its mount frame sits at the origin.

Problem this solves
-------------------
When you export a gripper by selecting it out of a larger assembled STEP
(e.g. UR16e + gripper), the saved USD keeps the *assembly* world coordinates:
the mesh points are baked at the arm-base-relative pose, so the prim's local
origin ends up at the UR base, far from the gripper itself. Attaching such a
prim under `tool0` with identity puts it in the wrong place, and its pivot is
useless for finger rigging.

This tool rebases the geometry into a clean tool frame: it bakes a rigid
transform into every mesh's points (and normals), then clears all xform ops,
so that a chosen anchor (the coupling/mount face) lands at (0,0,0) and,
optionally, a chosen approach axis is rotated onto +Z. The result is a
"pre-aligned" tool that attaches to `tool0` with identity.

Because it edits pure USD it runs at L1 (no Kit boot):

    PKG=/isaac-sim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311
    PYTHONPATH=$PKG LD_LIBRARY_PATH=$PKG/bin /isaac-sim/python.sh \
        ee_template/recenter_ee.py --in <gripper.usd> --report

Modes to place the new origin (choose one):
  --to bbox-center                 AABB center
  --to bbox-face --axis -y         center of the AABB face on that side (the
                                   mount/coupling face; default axis -y)
  --to point --point x,y,z         an explicit source-world point
  --to ref --ref-usd U --ref-prim P  align fully to another prim's world frame
                                   (e.g. the URDF tool0) -> exact pre-alignment

Optional orientation:
  --approach +y|-y|+x|-x|+z|-z     rotate so this source-world axis becomes +Z
                                   (the tool approach). Omit to keep orientation.

Optional unit fix:
  --set-meters-per-unit 1.0        rewrite stage metersPerUnit WITHOUT scaling
                                   points (use when a CAD import mis-tagged a
                                   metre-magnitude asset as cm/mm).
"""
import argparse
import sys

from pxr import Usd, UsdGeom, Gf, Vt

AXES = {
    "+x": Gf.Vec3d(1, 0, 0), "-x": Gf.Vec3d(-1, 0, 0),
    "+y": Gf.Vec3d(0, 1, 0), "-y": Gf.Vec3d(0, -1, 0),
    "+z": Gf.Vec3d(0, 0, 1), "-z": Gf.Vec3d(0, 0, -1),
}


def world_aabb(stage, root):
    """Tight world AABB from ACTUAL points (not authored extents, which CAD
    imports often pad -> that padding would offset a bbox-face anchor)."""
    xf = UsdGeom.XformCache(Usd.TimeCode.Default())
    mn = [1e30, 1e30, 1e30]
    mx = [-1e30, -1e30, -1e30]
    found = False
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.PointBased):
            continue
        pts = UsdGeom.PointBased(prim).GetPointsAttr().Get()
        if not pts:
            continue
        m = xf.GetLocalToWorldTransform(prim)
        for p in pts:
            w = m.Transform(Gf.Vec3d(p[0], p[1], p[2]))
            for j in range(3):
                mn[j] = min(mn[j], w[j]); mx[j] = max(mx[j], w[j])
            found = True
    if not found:
        return Gf.Vec3d(0, 0, 0), Gf.Vec3d(0, 0, 0)
    return Gf.Vec3d(*mn), Gf.Vec3d(*mx)


def report(stage, root):
    mn, mx = world_aabb(stage, root)
    ctr = [(mn[i] + mx[i]) / 2 for i in range(3)]
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    print(f"metersPerUnit = {mpu}  (1 unit = {mpu} m)")
    print(f"world AABB min   : {tuple(round(v,4) for v in mn)}")
    print(f"world AABB max   : {tuple(round(v,4) for v in mx)}")
    print(f"world AABB size  : {tuple(round(mx[i]-mn[i],4) for i in range(3))}  units")
    print(f"world AABB size  : {tuple(round((mx[i]-mn[i])*mpu,4) for i in range(3))}  metres")
    print(f"world AABB center: {tuple(round(v,4) for v in ctr)}")
    print("\nface-center candidates (pass to --to bbox-face --axis <ax>):")
    for ax, v in AXES.items():
        p = list(ctr)
        i = [abs(v[0]), abs(v[1]), abs(v[2])].index(1)
        p[i] = mx[i] if v[i] > 0 else mn[i]
        print(f"  {ax}: {tuple(round(c,4) for c in p)}")
    print("\nper-mesh point centroids (smallest along an axis = likely mount side):")
    xf = UsdGeom.XformCache(Usd.TimeCode.Default())
    rows = []
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            if not pts:
                continue
            m = xf.GetLocalToWorldTransform(prim)
            acc = Gf.Vec3d(0, 0, 0)
            for p in pts:
                acc += m.Transform(Gf.Vec3d(p[0], p[1], p[2]))
            acc /= len(pts)
            rows.append((prim.GetName(), len(pts), acc))
    for name, n, c in rows:
        print(f"  {name:20s} n={n:6d}  centroid={tuple(round(v,4) for v in c)}")


def build_frame(stage, root, args):
    """Return F (Gf.Matrix4d): maps new-local -> source-world. new = world * F^-1."""
    rot = Gf.Rotation()  # identity
    if args.approach:
        rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), AXES[args.approach])  # +Z -> approach

    if args.to == "ref":
        ref = Usd.Stage.Open(args.ref_usd)
        prim = ref.GetPrimAtPath(args.ref_prim)
        if not prim or not prim.IsValid():
            sys.exit(f"ref prim not found: {args.ref_prim}")
        F = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
        return F  # full frame from the reference (tool0), rotation included

    mn, mx = world_aabb(stage, root)
    ctr = Gf.Vec3d(*[(mn[i] + mx[i]) / 2 for i in range(3)])
    if args.to == "bbox-center":
        A = ctr
    elif args.to == "bbox-face":
        v = AXES[args.axis]
        i = [abs(v[0]), abs(v[1]), abs(v[2])].index(1)
        A = Gf.Vec3d(ctr)
        A[i] = mx[i] if v[i] > 0 else mn[i]
    elif args.to == "point":
        A = Gf.Vec3d(*[float(x) for x in args.point.split(",")])
    else:
        sys.exit(f"unknown --to {args.to}")

    F = Gf.Matrix4d().SetRotate(rot)
    F.SetTranslateOnly(A)
    return F


def rebase(stage, F):
    """Bake world*F^-1 into every PointBased prim, then clear all xform ops."""
    Finv = F.GetInverse()
    xf = UsdGeom.XformCache(Usd.TimeCode.Default())
    n_mesh = 0
    for prim in stage.Traverse():
        pb = UsdGeom.PointBased(prim)
        if not pb or not prim.IsA(UsdGeom.PointBased):
            continue
        pts = pb.GetPointsAttr().Get()
        if not pts:
            continue
        M = xf.GetLocalToWorldTransform(prim)      # local -> source world
        comp = M * Finv                            # local -> new local
        new = Vt.Vec3fArray(len(pts))
        mn = [1e30, 1e30, 1e30]
        mx = [-1e30, -1e30, -1e30]
        for k, p in enumerate(pts):
            w = comp.Transform(Gf.Vec3d(p[0], p[1], p[2]))
            new[k] = Gf.Vec3f(w[0], w[1], w[2])
            for j in range(3):
                mn[j] = min(mn[j], w[j]); mx[j] = max(mx[j], w[j])
        pb.GetPointsAttr().Set(new)
        na = pb.GetNormalsAttr()
        normals = na.Get()
        if normals:
            nn = Vt.Vec3fArray(len(normals))
            for k, nrm in enumerate(normals):
                d = comp.TransformDir(Gf.Vec3d(nrm[0], nrm[1], nrm[2])).GetNormalized()
                nn[k] = Gf.Vec3f(d[0], d[1], d[2])
            na.Set(nn)
        pb.GetExtentAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*mn), Gf.Vec3f(*mx)]))
        n_mesh += 1

    # geometry now holds absolute new-local coords -> every xform must be identity
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Xformable):
            UsdGeom.Xformable(prim).ClearXformOpOrder()
    return n_mesh


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--in", dest="inp", required=True)
    ap.add_argument("--out", dest="out")
    ap.add_argument("--root", help="prim to treat as tool root (default: defaultPrim)")
    ap.add_argument("--report", action="store_true", help="print geometry stats and exit")
    ap.add_argument("--to", choices=["bbox-center", "bbox-face", "point", "ref"],
                    default="bbox-face")
    ap.add_argument("--axis", choices=list(AXES), default="-y",
                    help="face side for --to bbox-face (mount/coupling face)")
    ap.add_argument("--point", help="x,y,z for --to point")
    ap.add_argument("--approach", choices=list(AXES),
                    help="source-world axis to rotate onto +Z (tool approach)")
    ap.add_argument("--ref-usd")
    ap.add_argument("--ref-prim")
    ap.add_argument("--set-meters-per-unit", type=float,
                    help="rewrite metersPerUnit without scaling points")
    args = ap.parse_args()

    stage = Usd.Stage.Open(args.inp)
    root = stage.GetPrimAtPath(args.root) if args.root else stage.GetDefaultPrim()
    if not root or not root.IsValid():
        root = stage.GetPseudoRoot()

    if args.report:
        report(stage, root)
        return

    F = build_frame(stage, root, args)
    print("new-origin frame (source-world translation):",
          tuple(round(v, 4) for v in F.ExtractTranslation()))
    n = rebase(stage, F)
    if args.set_meters_per_unit is not None:
        UsdGeom.SetStageMetersPerUnit(stage, args.set_meters_per_unit)
        print("metersPerUnit set to", args.set_meters_per_unit)

    mn, mx = world_aabb(stage, root)
    print(f"rebased {n} meshes. new AABB min={tuple(round(v,4) for v in mn)} "
          f"max={tuple(round(v,4) for v in mx)}")

    out = args.out or args.inp.replace(".usd", "_aligned.usd")
    stage.GetRootLayer().Export(out)
    print("wrote", out)


if __name__ == "__main__":
    main()
