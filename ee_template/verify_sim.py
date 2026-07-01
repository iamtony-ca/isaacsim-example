#!/usr/bin/env python3
"""Stage-3 behavioral verify (L2 / headless Kit) — replaces GUI Play eyeballing.

Boots a headless Isaac Sim, loads an actuated EE USD, drives the tool joints
open->closed, and ASSERTS the behavior instead of watching it:
  * every driven joint converges to its commanded target (within tol)
  * fingers actually move (non-trivial travel)
  * the base link does not explode / go NaN (articulation stays sane)

Deterministic, headless, CI-able — the whole point of the template.

Run (boots Kit, needs GPU):
  /isaac-sim/python.sh ee_template/verify_sim.py ee_template/out/ur16e_2f85_actuated.usd
Optional: --joints finger_left_joint,finger_right_joint  (default: auto = joints named *finger*)
          --close-deg 25   --settle 60   --steps 200
Exit code 0 = PASS, 1 = FAIL.
"""
import sys, os, math, argparse, functools, traceback

print = functools.partial(print, flush=True)   # Kit hard-exits; unbuffer so results survive

ap = argparse.ArgumentParser()
ap.add_argument("usd")
ap.add_argument("--artic", default="/ur16e")
ap.add_argument("--joints", default="", help="comma list; default = auto-detect *finger* joints")
ap.add_argument("--close-deg", type=float, default=25.0)
ap.add_argument("--settle", type=int, default=60, help="steps to let arm settle before driving fingers")
ap.add_argument("--steps", type=int, default=200, help="steps to reach the closed target")
ap.add_argument("--tol-deg", type=float, default=3.0)
args = ap.parse_args()

# --- boot headless Kit FIRST (must precede any omni/isaacsim import) ---
from isaacsim import SimulationApp
sim_app = SimulationApp({"headless": True})

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction


def fail(msg):
    print("  verify_sim FAIL:", msg); sim_app.close(); sys.exit(1)


def main():
    omni.usd.get_context().open_stage(os.path.abspath(args.usd))
    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60.0)
    world.reset()

    art = SingleArticulation(prim_path=args.artic, name="ee")
    art.initialize()
    names = list(art.dof_names)
    print("  dof_names:", names)

    # pick the tool joints to exercise
    if args.joints:
        joints = [j.strip() for j in args.joints.split(",")]
    else:
        joints = [n for n in names if "finger" in n.lower()]
    if not joints:
        fail("no tool joints found to drive (looked for *finger*; pass --joints)")
    idx = [names.index(j) for j in joints]
    print("  driving joints:", joints, "-> dof idx", idx)

    # let the arm settle at its baked pose
    for _ in range(args.settle):
        world.step(render=False)

    q_open = art.get_joint_positions()
    base_open, _ = art.get_world_pose()
    tgt = math.radians(args.close_deg)

    # command closed on the tool joints only
    q_cmd = np.array(art.get_joint_positions(), dtype=float)
    for i in idx:
        q_cmd[i] = tgt
    art.apply_action(ArticulationAction(joint_positions=q_cmd))   # position-drive targets
    for _ in range(args.steps):
        world.step(render=False)

    q_closed = art.get_joint_positions()
    base_closed, _ = art.get_world_pose()

    ok = True
    if not np.all(np.isfinite(q_closed)):
        fail("joint positions went NaN/inf -> articulation blew up")
    base_drift = float(np.linalg.norm(np.array(base_closed) - np.array(base_open)))
    if base_drift > 0.05:
        print("  base drift %.4f m (>0.05) -> unstable" % base_drift); ok = False
    else:
        print("  base drift %.4f m  OK" % base_drift)

    for j, i in zip(joints, idx):
        opened, closed = math.degrees(q_open[i]), math.degrees(q_closed[i])
        travel = abs(closed - opened)
        conv = abs(closed - args.close_deg) <= args.tol_deg
        moved = travel > 1.0
        flag = "OK" if (conv and moved) else "BAD"
        print("  %-16s open=%6.2f  closed=%6.2f  travel=%5.2f  -> target %.1f  [%s]"
              % (j, opened, closed, travel, args.close_deg, flag))
        ok = ok and conv and moved

    print("  verify_sim:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


try:
    rc = main()
except Exception:
    traceback.print_exc(); rc = 2
finally:
    sim_app.close()
sys.exit(rc)
