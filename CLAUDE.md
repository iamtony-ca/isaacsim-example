# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`test_ws` is an NVIDIA Isaac Sim 5.1 workspace for building a **fixed UR16e arm + swappable
end-effector** into a movable, simulatable robot, starting from CAD (STEP) and the UR16e URDF.
It is part asset workspace, part small codebase: the reusable pipeline lives in `ee_template/`
(config-driven, headless) and `scripts/` (legacy Step-4a rig). No package manifest; "work here"
means editing USD assets and running the pipeline scripts.

Run all commands from this `test_ws/` directory (paths in scripts/configs are relative to it).

## Layout

- `docs/` — `UR16e_custom_gripper_tutorial.md` (step-by-step build-up) + `imgs/`.
- `robot/` — `ur16e.urdf` (source) and `ur16e/` URDF-import USD output (generated, gitignored).
- `scripts/rig_gripper_fingers.py` — legacy Step-4a finger-actuation rig for the 2F-85 gripper.
- `ee_template/` — the reusable headless pipeline. `build_ee.py` (rig+structural verify, L1),
  `verify_sim.py` (headless sim behavioral verify, L2), `configs/*.yaml`, `README.md`, `out/`.
- `cad/` — large vendor CAD sources (gitignored; see `cad/README.md`).
- `assets/` — generated USD stages (gitignored; see `assets/README.md`).

This is a **lean repo**: CAD and generated USD binaries are not committed (`.gitignore`). Only
source (scripts, configs, `ur16e.urdf`, docs, images) is tracked.

## Two execution levels

The Isaac Sim 5.1 runtime lives at `/isaac-sim`. Use its bundled Python, not a system Python.

- **L1 pure USD** — structure inspection & physics authoring, no Kit boot. The bundled
  `python.sh` has NO `pxr` on its own; use the extscache USD libs:
  ```bash
  PKG=/isaac-sim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311
  PYTHONPATH=$PKG LD_LIBRARY_PATH=$PKG/bin /isaac-sim/python.sh <script.py>
  ```
- **L2 headless Kit** — CAD/URDF import, physics sim, ROS 2. Boots `SimulationApp(headless=True)`
  via plain `/isaac-sim/python.sh <script.py>` (needs GPU; ~6 s boot). `isaacsim.core.*` is the
  5.x namespace (`omni.isaac.*` deprecated).

```bash
# Inspect / convert a USD crate to ASCII (.usda) for reading/diffing (L1)
PYTHONPATH=$PKG LD_LIBRARY_PATH=$PKG/bin /isaac-sim/python.sh -c \
  "from pxr import Usd; print(Usd.Stage.Open('assets/ur16e_gripper_base.usd').ExportToString())" | less

# GUI on a stage
/isaac-sim/isaac-sim.sh          # then File > Open
```

## Conventions

- Edit USD by script through `pxr.Usd`/`pxr.UsdGeom`/`pxr.UsdPhysics`, or in the GUI — USD crate
  files are binary; do not hand-edit the bytes. Convert to `.usda` to inspect/diff.
- New end-effector = a new `ee_template/configs/*.yaml` (module tree), not new code. Keep the
  driver (`build_ee.py`) generic; per-tool specifics live in the config.
- Finger/tool joints attach with `body0 = wrist_3_link`, so they join the `/ur16e` **articulation**
  — in GUI Play, select the UR16e articulation (not the tool link) to drive them.
- Pipeline changes: verify with `build_ee.py` (structural) then `verify_sim.py` (behavioral).
