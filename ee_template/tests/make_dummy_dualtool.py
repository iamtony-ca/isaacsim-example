#!/usr/bin/env python3
"""Generate a SYNTHETIC dual-tool end-effector on top of the real UR16e arm, so the
generalized template path (assembly_node split, tree topology, fixed+prismatic+revolute
joints, semantic frames) can be validated end-to-end WITHOUT the real OnRobot STEP.

Builds named assembly nodes (each with a cube solid as a direct Mesh child) under tool0,
mimicking a Y quick-changer: QuickChanger→{Damper, HEX_A→{FG14_A_L, FG14_A_R}, HEX_B→Screwdriver}.
Output (gitignored) feeds ee_template/configs/dummy_dualtool.yaml. Run from test_ws/ root:

  PKG=/isaac-sim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311
  PYTHONPATH=$PKG LD_LIBRARY_PATH=$PKG/bin /isaac-sim/python.sh ee_template/tests/make_dummy_dualtool.py
"""
from pxr import Usd, UsdGeom, Gf

ARM = "assets/ur16e_gripper_base.usd"
OUT = "assets/ur16e_dummy_dualtool.usd"
TOOL0 = "/ur16e/wrist_3_link/flange/tool0"
EE = TOOL0 + "/dual_tool"          # = tool_frame in the config


def add_cube(stage, path, size=20.0):     # size in mm (frame is mm-scaled, see main)
    m = UsdGeom.Mesh.Define(stage, path)
    s = size / 2.0
    pts = [(-s,-s,-s),(s,-s,-s),(s,s,-s),(-s,s,-s),(-s,-s,s),(s,-s,s),(s,s,s),(-s,s,s)]
    m.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    m.CreateFaceVertexCountsAttr([4]*6)
    m.CreateFaceVertexIndicesAttr([0,3,2,1, 4,5,6,7, 0,1,5,4, 2,3,7,6, 1,2,6,5, 0,4,7,3])
    return m


def node(stage, name, t):
    """A named assembly node (Xform) holding one cube solid as a direct Mesh child.
    `t` is in mm — the EE frame is mm-scaled (0.001) so config pivot_mm/origin_mm match."""
    p = EE + "/" + name
    x = UsdGeom.Xform.Define(stage, p)
    UsdGeom.Xformable(x).AddTranslateOp().Set(Gf.Vec3f(*t))   # mm (EE frame is 0.001-scaled)
    add_cube(stage, p + "/solid")
    return p


def main():
    st = Usd.Stage.Open(Usd.Stage.Open(ARM).Flatten())
    UsdGeom.Xform.Define(st, TOOL0)     # ensure tool0 exists
    ee = UsdGeom.Xform.Define(st, EE)
    # Replicate the CAD convention: tool-local space is MILLIMETRES with a 0.001 unitsResolve
    # scale, so g2w maps mm->world and config pivot_mm/origin_mm are correct.
    UsdGeom.Xformable(ee).AddScaleOp().Set(Gf.Vec3f(0.001, 0.001, 0.001))
    # name -> tool-local position (mm); +Z approach, two branches on ±X
    node(st, "QuickChanger", (0,   0, 20))
    node(st, "Damper",       (0,   0, 35))
    node(st, "HEX_A",        (45,  0, 50))
    node(st, "FG14_A_L",     (60,  0, 85))
    node(st, "FG14_A_R",     (30,  0, 85))
    node(st, "HEX_B",        (-45, 0, 50))
    node(st, "Screwdriver",  (-45, 0, 90))
    st.GetRootLayer().Export(OUT)
    print("exported", OUT)


main()
