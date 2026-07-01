#!/usr/bin/env python3
# Reproducible gripper-finger actuation rig for UR16e (Approach A).
# Arm is fixed (UR16e URDF); gripper/cam swap freely. Per-gripper specifics live in CONFIG.
# Run with isaac python.sh + omni.usd.libs pxr on PYTHONPATH/LD_LIBRARY_PATH.
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

# ======================= CONFIG (per-gripper) =======================
CONFIG = {
    "src":  "assets/ur16e_2f85_rigged_my.usd",   # arm + gripper rigidly attached (Step 2 output); run from test_ws/
    "out":  "assets/ur16e_2f85_actuated.usd",
    "gripper_root": "/ur16e/wrist_3_link/flange/tool0/gripper_2f85_tool",
    "artic_root":   "/ur16e",            # new finger links go here (siblings of arm links)
    "mount_link":   "/ur16e/wrist_3_link",  # body0 of finger joints (gripper base is part of this link)
    # gripper-local frame is mm; +Z=approach, fingers spread along X, symmetry ~X=0.
    # segmentation: assign a gripper solid (Mesh) to a finger by local centroid X.
    "solid_parent_tag": "7912_",         # the gripper assembly node (vs coupling 7911); meshes under it are solids
    "fingers": [
        {"name": "finger_left",  "cx_min": -1e9, "cx_max": -10.0,
         "pivot_mm": (-13.0, 0.0, 58.0), "rot_axis_local": (0,1,0), "axis_sign": +1,
         "limits_deg": (0.0, 45.0)},
        {"name": "finger_right", "cx_min":  10.0, "cx_max":  1e9,
         "pivot_mm": ( 13.0, 0.0, 58.0), "rot_axis_local": (0,1,0), "axis_sign": -1,
         "limits_deg": (0.0, 45.0)},
    ],
    "finger_mass_kg": 0.1,
    "drive": {"type": "acceleration", "stiffness": 2.0e4, "damping": 2.0e3,
              "max_force": 1.0e3, "target_deg": 0.0},
    "collider_approx": "convexHull",
}
# ====================================================================

def vec(t): return Gf.Vec3d(*t)

def main(cfg):
    src = Usd.Stage.Open(cfg["src"])
    flat = src.Flatten()                       # bake payload -> editable local meshes
    st = Usd.Stage.Open(flat)
    tc = Usd.TimeCode.Default()

    grip = st.GetPrimAtPath(cfg["gripper_root"])
    g2w  = UsdGeom.Xformable(grip).ComputeLocalToWorldTransform(tc)
    w2g  = g2w.GetInverse()
    root = st.GetPrimAtPath(cfg["artic_root"])
    Wroot_inv = UsdGeom.Xformable(root).ComputeLocalToWorldTransform(tc).GetInverse()

    # the gripper assembly node (its child meshes = solids)
    gnode = None
    for p in Usd.PrimRange(grip):
        if cfg["solid_parent_tag"] in p.GetName() and p.GetParent().GetName() == grip.GetName():
            gnode = p; break
    solids = [p for p in Usd.PrimRange(gnode) if p.GetTypeName() == "Mesh"]

    bbc = UsdGeom.BBoxCache(tc, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    def gcx(p):
        wb = bbc.ComputeWorldBound(p).ComputeAlignedRange(); mn, mx = wb.GetMin(), wb.GetMax()
        xs = [w2g.Transform(Gf.Vec3d(x, y, z))[0]
              for x in (mn[0], mx[0]) for y in (mn[1], mx[1]) for z in (mn[2], mx[2])]
        return (min(xs) + max(xs)) / 2

    # classify
    assign = {f["name"]: [] for f in cfg["fingers"]}
    for p in solids:
        cx = gcx(p)
        for f in cfg["fingers"]:
            if f["cx_min"] <= cx <= f["cx_max"]:
                assign[f["name"]].append(p); break
    print("segmentation:", {k: len(v) for k, v in assign.items()}, "of", len(solids), "solids")

    worldmat = {p.GetPath().pathString: UsdGeom.Xformable(p).ComputeLocalToWorldTransform(tc)
                for v in assign.values() for p in v}

    # reparent finger solids under new link Xforms (namespace edit on layer)
    edit = Sdf.BatchNamespaceEdit()
    for f in cfg["fingers"]:
        lp = cfg["artic_root"] + "/" + f["name"]
        UsdGeom.Xform.Define(st, lp)
        for i, p in enumerate(assign[f["name"]]):
            edit.Add(p.GetPath(), Sdf.Path(lp + "/solid_%d" % i))
    assert st.GetRootLayer().Apply(edit), "namespace edit failed"

    for f in cfg["fingers"]:
        lp = cfg["artic_root"] + "/" + f["name"]
        link = st.GetPrimAtPath(lp)
        # bake world xform into each moved solid; link stays identity under /ur16e
        for i, p in enumerate(assign[f["name"]]):
            sp = st.GetPrimAtPath(lp + "/solid_%d" % i)
            local = Wroot_inv * worldmat[p.GetPath().pathString]
            xf = UsdGeom.Xformable(sp); xf.ClearXformOpOrder(); xf.AddTransformOp().Set(local)
            UsdPhysics.CollisionAPI.Apply(sp.GetPrim())
            mca = UsdPhysics.MeshCollisionAPI.Apply(sp.GetPrim())
            mca.CreateApproximationAttr().Set(cfg["collider_approx"])
        # rigid body + mass on the link
        UsdPhysics.RigidBodyAPI.Apply(link)
        m = UsdPhysics.MassAPI.Apply(link); m.CreateMassAttr().Set(cfg["finger_mass_kg"])

    # joints
    Mb0 = UsdGeom.Xformable(st.GetPrimAtPath(cfg["mount_link"])).ComputeLocalToWorldTransform(tc)
    q0 = Mb0.ExtractRotationQuat()
    for f in cfg["fingers"]:
        lp = cfg["artic_root"] + "/" + f["name"]
        Mb1 = UsdGeom.Xformable(st.GetPrimAtPath(lp)).ComputeLocalToWorldTransform(tc)
        q1 = Mb1.ExtractRotationQuat()
        # desired world rotation axis = gripper-local axis through g2w, with sign
        d = g2w.TransformDir(vec(f["rot_axis_local"]) * f["axis_sign"]); d = d.GetNormalized()
        q_align = Gf.Rotation(Gf.Vec3d(1, 0, 0), d).GetQuat()   # map joint-frame X -> d
        lr0 = (q0.GetInverse() * q_align); lr1 = (q1.GetInverse() * q_align)
        pivot_w = g2w.Transform(vec(f["pivot_mm"]))
        lp0 = Mb0.GetInverse().Transform(pivot_w)
        lp1 = Mb1.GetInverse().Transform(pivot_w)
        jpath = cfg["artic_root"] + "/" + f["name"] + "_joint"
        j = UsdPhysics.RevoluteJoint.Define(st, jpath)
        j.CreateBody0Rel().SetTargets([cfg["mount_link"]])
        j.CreateBody1Rel().SetTargets([lp])
        j.CreateAxisAttr().Set("X")
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(lp0)); j.CreateLocalRot0Attr().Set(Gf.Quatf(lr0))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(lp1)); j.CreateLocalRot1Attr().Set(Gf.Quatf(lr1))
        j.CreateLowerLimitAttr().Set(f["limits_deg"][0])
        j.CreateUpperLimitAttr().Set(f["limits_deg"][1])
        drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        dc = cfg["drive"]
        drv.CreateTypeAttr().Set(dc["type"])
        drv.CreateStiffnessAttr().Set(dc["stiffness"])
        drv.CreateDampingAttr().Set(dc["damping"])
        drv.CreateMaxForceAttr().Set(dc["max_force"])
        drv.CreateTargetPositionAttr().Set(dc["target_deg"])
        print(" %-13s axis_world=%s pivot_world=%s" %
              (f["name"], [round(v,3) for v in d], [round(v,4) for v in pivot_w]))

    st.GetRootLayer().Export(cfg["out"])
    print("exported", cfg["out"])

main(CONFIG)
