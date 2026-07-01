#!/usr/bin/env python3
"""Headless end-effector builder for the FIXED UR16e + VARIABLE tool template.

Goal: UR16e is fixed; the end-effector (grippers, F/T sensors, screwdrivers, dual
tool-changers) varies constantly. A new tool = a new YAML config, not new code.

Pipeline stages (all headless):
  0. convert   STEP -> USD               [L2 / Kit; STUB unless source.rigged_usd given]
  1. attach    tool -> tool0 (identity)  [L2 / Kit; STUB unless source.rigged_usd given]
  2. rig       module tree -> links+joints+drives   [L1 / pure pxr; IMPLEMENTED]
  3. verify    headless sim smoke-test    [L2 / Kit; STUB -> structural check for now]

Run (pure-pxr path, no Kit boot):
  PKG=/isaac-sim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311
  PYTHONPATH=$PKG LD_LIBRARY_PATH=$PKG/bin /isaac-sim/python.sh \
      ee_template/build_ee.py ee_template/configs/robotiq_2f85.yaml
"""
import os, sys, yaml
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf


# ----------------------------------------------------------------------------- helpers
def vec(t):            return Gf.Vec3d(*t)
def die(msg):          print("ERROR:", msg); sys.exit(1)


class Ctx:
    """Frames shared by every module: tool-local frame, articulation root, mount body."""
    def __init__(self, st, cfg):
        self.st, self.cfg = st, cfg
        self.tc = Usd.TimeCode.Default()
        self.tool = st.GetPrimAtPath(cfg["source"]["tool_frame"])
        if not self.tool.IsValid():
            die("tool_frame not found: " + cfg["source"]["tool_frame"])
        self.g2w = UsdGeom.Xformable(self.tool).ComputeLocalToWorldTransform(self.tc)
        self.w2g = self.g2w.GetInverse()
        self.root = st.GetPrimAtPath(cfg["artic_root"])
        self.Wroot_inv = UsdGeom.Xformable(self.root).ComputeLocalToWorldTransform(self.tc).GetInverse()
        self.bbc = UsdGeom.BBoxCache(self.tc, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

    def link_path(self, name):  return self.cfg["artic_root"] + "/" + name

    def parent_link_path(self, module):
        p = module["parent"]
        return self.cfg["mount_link"] if p == "mount_link" else self.link_path(p)

    def gcx(self, prim):
        """centroid X of a prim in tool-local (mm) frame."""
        wb = self.bbc.ComputeWorldBound(prim).ComputeAlignedRange(); mn, mx = wb.GetMin(), wb.GetMax()
        xs = [self.w2g.Transform(Gf.Vec3d(x, y, z))[0]
              for x in (mn[0], mx[0]) for y in (mn[1], mx[1]) for z in (mn[2], mx[2])]
        return (min(xs) + max(xs)) / 2


# ----------------------------------------------------------------------------- selection
def select_solids(ctx, sel):
    """Return the Mesh prims belonging to one module. Extensible by `by:`."""
    by = sel["by"]
    if by == "centroid_x":
        # meshes under the assembly node tagged `parent_tag`, filtered by tool-local centroid X.
        tool = ctx.tool
        gnode = None
        for p in Usd.PrimRange(tool):
            if sel["parent_tag"] in p.GetName() and p.GetParent().GetName() == tool.GetName():
                gnode = p; break
        if gnode is None:
            die("centroid_x: parent_tag node not found: " + sel["parent_tag"])
        # float() guards the PyYAML quirk where `1.0e9` (no exponent sign) parses as str.
        cx_min, cx_max = float(sel["cx_min"]), float(sel["cx_max"])
        out = []
        for p in Usd.PrimRange(gnode):
            if p.GetTypeName() != "Mesh":
                continue
            if cx_min <= ctx.gcx(p) <= cx_max:
                out.append(p)
        return out
    if by == "assembly_node":
        # Robust split for complex multi-tool assemblies: pick a named assembly node and take
        # its own solids. Default = the node's DIRECT Mesh children (sub-tools live under child
        # Xform nodes and are claimed by their own modules). `subtree: true` grabs all descendant
        # meshes (for a leaf part whose solids are nested a few levels deep).
        node_name = sel["node"]
        node = None
        for p in Usd.PrimRange(ctx.tool):
            if p.GetName() == node_name:
                node = p; break
        if node is None:
            die("assembly_node: node '%s' not found under %s" % (node_name, ctx.tool.GetPath()))
        if sel.get("subtree", False):
            return [p for p in Usd.PrimRange(node) if p.GetTypeName() == "Mesh"]
        return [c for c in node.GetChildren() if c.GetTypeName() == "Mesh"]
    die("unknown select.by: " + by)


# ----------------------------------------------------------------------------- rigging
def build_link(ctx, module, solids):
    """Reparent `solids` under a new link, bake world xform, add rigid body + colliders."""
    st = ctx.st
    lp = ctx.link_path(module["name"])
    UsdGeom.Xform.Define(st, lp)
    worldmat = {p.GetPath().pathString:
                UsdGeom.Xformable(p).ComputeLocalToWorldTransform(ctx.tc) for p in solids}
    edit = Sdf.BatchNamespaceEdit()
    for i, p in enumerate(solids):
        edit.Add(p.GetPath(), Sdf.Path(lp + "/solid_%d" % i))
    if not st.GetRootLayer().Apply(edit):
        die("namespace edit failed for module " + module["name"])
    approx = module["body"].get("collider", "convexHull")
    for i, p in enumerate(solids):
        sp = st.GetPrimAtPath(lp + "/solid_%d" % i)
        local = ctx.Wroot_inv * worldmat[p.GetPath().pathString]
        xf = UsdGeom.Xformable(sp); xf.ClearXformOpOrder(); xf.AddTransformOp().Set(local)
        UsdPhysics.CollisionAPI.Apply(sp.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(sp.GetPrim()).CreateApproximationAttr().Set(approx)
    link = st.GetPrimAtPath(lp)
    UsdPhysics.RigidBodyAPI.Apply(link)
    UsdPhysics.MassAPI.Apply(link).CreateMassAttr().Set(float(module["body"]["mass_kg"]))
    return lp


def _anchor(ctx, body0_path, body1_path, point_w, axis_w):
    """Local pos/rot for both joint bodies so their frames coincide at zero, joint-X -> axis_w."""
    Mb0 = UsdGeom.Xformable(ctx.st.GetPrimAtPath(body0_path)).ComputeLocalToWorldTransform(ctx.tc)
    Mb1 = UsdGeom.Xformable(ctx.st.GetPrimAtPath(body1_path)).ComputeLocalToWorldTransform(ctx.tc)
    q0, q1 = Mb0.ExtractRotationQuat(), Mb1.ExtractRotationQuat()
    q_align = Gf.Rotation(Gf.Vec3d(1, 0, 0), axis_w).GetQuat()   # map joint-frame X -> axis_w
    lr0, lr1 = q0.GetInverse() * q_align, q1.GetInverse() * q_align
    lp0 = Gf.Vec3f(Mb0.GetInverse().Transform(point_w))
    lp1 = Gf.Vec3f(Mb1.GetInverse().Transform(point_w))
    return lp0, Gf.Quatf(lr0), lp1, Gf.Quatf(lr1)


def build_joint(ctx, module, body1_path):
    """Author the module's joint (revolute | prismatic | fixed) + drive."""
    st, j_cfg = ctx.st, module.get("joint", {"type": "fixed"})
    jtype = j_cfg["type"]
    body0_path = ctx.parent_link_path(module)
    jpath = ctx.link_path(module["name"]) + "_joint"

    if jtype == "fixed":
        j = UsdPhysics.FixedJoint.Define(st, jpath)
        j.CreateBody0Rel().SetTargets([body0_path]); j.CreateBody1Rel().SetTargets([body1_path])
        # weld at the current relative pose: joint frame = body1 origin, expressed in body0 frame.
        Mb0 = UsdGeom.Xformable(st.GetPrimAtPath(body0_path)).ComputeLocalToWorldTransform(ctx.tc)
        Mb1 = UsdGeom.Xformable(st.GetPrimAtPath(body1_path)).ComputeLocalToWorldTransform(ctx.tc)
        rel = Mb0.GetInverse() * Mb1
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(rel.ExtractTranslation()))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(rel.ExtractRotationQuat()))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
        return jpath

    # revolute / prismatic share anchor math; differ in joint prim + unit of limits/drive.
    axis_w = ctx.g2w.TransformDir(vec(j_cfg["axis_local"]) * j_cfg.get("axis_sign", 1)).GetNormalized()
    anchor_mm = j_cfg.get("pivot_mm", j_cfg.get("origin_mm", [0, 0, 0]))
    point_w = ctx.g2w.Transform(vec(anchor_mm))
    lp0, lr0, lp1, lr1 = _anchor(ctx, body0_path, body1_path, point_w, axis_w)

    if jtype == "revolute":
        j = UsdPhysics.RevoluteJoint.Define(st, jpath)
        lo, hi = j_cfg["limits"]                              # degrees
        drive_tok = "angular"
    elif jtype == "prismatic":
        j = UsdPhysics.PrismaticJoint.Define(st, jpath)
        lo, hi = [v / 1000.0 for v in j_cfg["limits_mm"]]     # mm -> meters
        drive_tok = "linear"
    else:
        die("unknown joint.type: " + jtype)

    j.CreateBody0Rel().SetTargets([body0_path]); j.CreateBody1Rel().SetTargets([body1_path])
    j.CreateAxisAttr().Set("X")                               # real axis carried by localRot
    j.CreateLocalPos0Attr().Set(lp0); j.CreateLocalRot0Attr().Set(lr0)
    j.CreateLocalPos1Attr().Set(lp1); j.CreateLocalRot1Attr().Set(lr1)
    if not j_cfg.get("continuous", False):
        j.CreateLowerLimitAttr().Set(float(lo)); j.CreateUpperLimitAttr().Set(float(hi))

    d = module.get("drive")
    if d:
        drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), drive_tok)
        drv.CreateTypeAttr().Set(d["type"])
        drv.CreateStiffnessAttr().Set(float(d["stiffness"]))
        drv.CreateDampingAttr().Set(float(d["damping"]))
        drv.CreateMaxForceAttr().Set(float(d["max_force"]))
        drv.CreateTargetPositionAttr().Set(float(d.get("target", 0)))
    return jpath


# ----------------------------------------------------------------------------- stages
def stage_source(cfg):
    src = cfg["source"]
    if "rigged_usd" in src:
        st = Usd.Stage.Open(src["rigged_usd"]).Flatten()      # bake payload -> editable meshes
        return Usd.Stage.Open(st)
    # STUB: headless CAD import (omni.kit.converter.cad) + identity attach to tool0.
    # Needs a Kit app (SimulationApp headless). Implement when we drop the GUI Step-0/2.
    raise NotImplementedError("source.step path needs headless Kit (CAD convert + attach) — TODO")


def stage_verify(st, cfg):
    """STUB for headless sim smoke-test; for now assert the structure is articulation-valid."""
    rbs = [p for p in st.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    roots = [p for p in st.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    joints = [p for p in st.Traverse() if "Joint" in str(p.GetTypeName())]
    print("  verify: rigidBodies=%d articulationRoots=%d joints=%d" % (len(rbs), len(roots), len(joints)))
    ok = len(roots) == 1
    # no rigid body nested under another rigid body (PhysX forbids it)
    for p in rbs:
        anc = p.GetParent()
        while anc and anc.IsValid() and anc.GetPath() != Sdf.Path("/"):
            if anc.HasAPI(UsdPhysics.RigidBodyAPI):
                print("  verify FAIL: nested rigid body", p.GetPath(), "under", anc.GetPath()); ok = False; break
            anc = anc.GetParent()
    # TODO(L2): boot SimulationApp(headless=True), drive each joint open/close,
    #           assert convergence + no interpenetration, optional camera PNG snapshot.
    print("  verify:", "PASS (structural)" if ok else "FAIL")
    return ok


def main(cfg_path):
    cfg = yaml.safe_load(open(cfg_path))
    print("== build_ee: %s ==" % cfg["name"])
    st = stage_source(cfg)                    # stages 0-1 (or reuse pre-attached USD)
    ctx = Ctx(st, cfg)

    print("-- stage rig: %d modules --" % len(cfg["modules"]))
    for module in cfg["modules"]:             # stage 2 (pure pxr)
        solids = select_solids(ctx, module["select"]) if "select" in module else []
        lp = build_link(ctx, module, solids)
        build_joint(ctx, module, lp)
        sem = module.get("frame", {}).get("semantic")
        if sem:                                # expose a semantic frame (ROS wrench / TCP) as a tag
            st.GetPrimAtPath(lp).CreateAttribute("ee:frame_semantic", Sdf.ValueTypeNames.Token).Set(sem)
        print("  module %-14s solids=%d  joint=%-9s parent=%-12s %s"
              % (module["name"], len(solids), module.get("joint", {}).get("type"),
                 module["parent"], ("frame=" + sem) if sem else ""))

    ok = stage_verify(st, cfg)                # stage 3
    out = cfg["out"]; os.makedirs(os.path.dirname(out), exist_ok=True)
    st.GetRootLayer().Export(out)
    print("exported", out, "\n== %s ==" % ("OK" if ok else "STRUCTURAL FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    if len(sys.argv) != 2:
        die("usage: build_ee.py <config.yaml>")
    sys.exit(main(sys.argv[1]))
