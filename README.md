# UR16e End-Effector Workspace (Isaac Sim 5.1)

Reproducible pipeline for staging a **fixed UR16e arm** with **swappable end-effectors**
(grippers, F/T sensors, screwdrivers, tool-changers) in NVIDIA Isaac Sim 5.1 — starting
from CAD (STEP) + the UR16e URDF, building up to a movable, simulatable robot.

## Layout
```
docs/          tutorial + images
  UR16e_custom_gripper_tutorial.md   step-by-step build-up (Korean)
robot/         ur16e.urdf (source) + ur16e/ URDF-import USD output (generated, gitignored)
scripts/       rig_gripper_fingers.py — legacy Step-4a finger-actuation rig (2F-85)
ee_template/   headless, config-driven EE template (the reusable pipeline)
  build_ee.py  verify_sim.py  configs/*.yaml  README.md
cad/           CAD sources (gitignored — see cad/README.md)
assets/        generated USD stages (gitignored — see assets/README.md)
```

## This is a lean repo
Large CAD (100s of MB) and generated USD binaries are **not committed** (`.gitignore`).
Only source is tracked: scripts, configs, `ur16e.urdf`, docs, images. To reproduce the
assets, fetch the CAD (`cad/README.md`) and run the pipeline (`assets/README.md`,
`ee_template/README.md`). Run all commands from this `test_ws/` directory.

## Where to start
- **Learn the build-up (with GUI):** `docs/UR16e_custom_gripper_tutorial.md` §0–§7.
- **Run it headless / swap end-effectors:** §8 of the tutorial and `ee_template/README.md`.

## Runtime
Isaac Sim 5.1 at `/isaac-sim`. Two execution levels:
- **L1 pure USD** (no Kit boot) — structure edits & physics authoring via the bundled
  `pxr` from extscache: `PYTHONPATH=$PKG LD_LIBRARY_PATH=$PKG/bin /isaac-sim/python.sh …`
  where `PKG=/isaac-sim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311`.
- **L2 headless Kit** — CAD/URDF import, physics sim, ROS 2 via `/isaac-sim/python.sh`
  (`SimulationApp(headless=True)`).
