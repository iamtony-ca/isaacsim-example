#!/usr/bin/env python3
"""Stage-3 behavioral verify (L2 / headless Kit) — replaces GUI Play eyeballing.

Boots a headless Isaac Sim, loads an actuated EE USD, and for every *tool* joint (any DOF
not part of the fixed UR16e arm) drives it to its upper limit and ASSERTS the behavior,
joint-type aware:
  * revolute joints reported/tuned in degrees, prismatic in millimetres
  * each driven joint converges to its commanded target (within tol)
  * it actually moves (non-trivial travel)
  * the base link does not explode / go NaN

Deterministic, headless, CI-able. Works for the 2F-85 gripper and the dual-tool EE alike.

  /isaac-sim/python.sh ee_template/verify_sim.py ee_template/out/ur16e_2f85_actuated.usd
  /isaac-sim/python.sh ee_template/verify_sim.py ee_template/out/ur16e_dummy_dualtool_actuated.usd
Optional: --joints a,b  (default = auto: every DOF not in the UR16e arm)
          --settle 60  --steps 220  --tol-deg 3  --tol-mm 1
Exit 0 = PASS, 1 = FAIL.
"""
import sys, os, math, argparse, functools, traceback

print = functools.partial(print, flush=True)   # Kit hard-exits; unbuffer so results survive

ap = argparse.ArgumentParser()
ap.add_argument("usd")
ap.add_argument("--artic", default="/ur16e")
ap.add_argument("--joints", default="", help="comma list; default = every DOF not in the UR16e arm")
ap.add_argument("--settle", type=int, default=60)
ap.add_argument("--steps", type=int, default=220)
ap.add_argument("--tol-deg", type=float, default=3.0)
ap.add_argument("--tol-mm", type=float, default=1.0)
args = ap.parse_args()

ARM_JOINTS = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}

# --- boot headless Kit FIRST (must precede any omni/isaacsim import) ---
from isaacsim import SimulationApp
sim_app = SimulationApp({"headless": True})

import numpy as np
import omni.usd
from pxr import UsdPhysics
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction


def fail(msg):
    print("  verify_sim FAIL:", msg); sim_app.close(); sys.exit(1)


def joint_plan(stage, name):
    """Return (kind, unit, to_native, from_native, upper_native, tol_native) for a DOF joint.
    Position drive is commanded in native units (rad for revolute, m for prismatic)."""
    prim = next((p for p in stage.Traverse()
                 if p.GetName() == name and "Joint" in str(p.GetTypeName())), None)
    if prim is None:
        fail("joint prim not found for dof '%s'" % name)
    if prim.GetTypeName() == "PhysicsPrismaticJoint":
        up = UsdPhysics.PrismaticJoint(prim).GetUpperLimitAttr().Get()        # metres
        return ("prismatic", "mm", (lambda v: v), (lambda v: v * 1000.0),
                up, args.tol_mm / 1000.0)
    up_deg = UsdPhysics.RevoluteJoint(prim).GetUpperLimitAttr().Get()          # degrees
    up = math.radians(up_deg) if up_deg is not None else None
    return ("revolute", "deg", math.radians, math.degrees, up, math.radians(args.tol_deg))


def main():
    omni.usd.get_context().open_stage(os.path.abspath(args.usd))
    stage = omni.usd.get_context().get_stage()
    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60.0)
    world.reset()

    art = SingleArticulation(prim_path=args.artic, name="ee")
    art.initialize()
    names = list(art.dof_names)
    print("  dof_names:", names)

    joints = ([j.strip() for j in args.joints.split(",")] if args.joints
              else [n for n in names if n not in ARM_JOINTS])
    if not joints:
        fail("no tool joints found (all DOFs are arm joints; pass --joints)")
    plans = {j: joint_plan(stage, j) for j in joints}
    idx = {j: names.index(j) for j in joints}
    print("  driving tool joints:", {j: plans[j][0] for j in joints})

    for _ in range(args.settle):
        world.step(render=False)
    q_open = np.array(art.get_joint_positions(), dtype=float)
    base_open, _ = art.get_world_pose()

    q_cmd = q_open.copy()
    for j in joints:
        kind, unit, to_nat, from_nat, upper, tol = plans[j]
        if upper is not None:
            q_cmd[idx[j]] = upper                 # drive to the closed / upper-limit pose
    art.apply_action(ArticulationAction(joint_positions=q_cmd))
    for _ in range(args.steps):
        world.step(render=False)

    q_closed = np.array(art.get_joint_positions(), dtype=float)
    base_closed, _ = art.get_world_pose()

    ok = True
    if not np.all(np.isfinite(q_closed)):
        fail("joint positions went NaN/inf -> articulation blew up")
    drift = float(np.linalg.norm(np.array(base_closed) - np.array(base_open)))
    print("  base drift %.4f m  %s" % (drift, "OK" if drift <= 0.05 else "UNSTABLE"))
    ok = ok and drift <= 0.05

    for j in joints:
        kind, unit, to_nat, from_nat, upper, tol = plans[j]
        o, c = q_open[idx[j]], q_closed[idx[j]]
        travel = abs(c - o)
        if upper is None:                          # continuous/velocity joint: just needs to move
            good = travel > to_nat(1.0) if unit == "deg" else travel > 0.001
            tstr = "spin"
        else:
            good = abs(c - upper) <= tol and travel > (to_nat(0.5) if unit == "deg" else 0.0005)
            tstr = "%.1f" % from_nat(upper)
        print("  %-18s open=%7.2f  closed=%7.2f  travel=%6.2f %s  -> target %s  [%s]"
              % (j, from_nat(o), from_nat(c), from_nat(travel), unit, tstr, "OK" if good else "BAD"))
        ok = ok and good

    print("  verify_sim:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


try:
    rc = main()
except Exception:
    traceback.print_exc(); rc = 2
finally:
    sim_app.close()
sys.exit(rc)
