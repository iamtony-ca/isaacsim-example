# Generated USD stages (not committed)

USD stages produced by the pipeline. Large binary crate files, excluded from git
(`.gitignore`) — regenerate from the CAD sources + scripts. Run all commands from the
`test_ws/` repo root.

```
assets/
  ur16e_2f85.usd            Step 0  — assembled static CAD, stood upright (no joints)
  ur16e_gripper_base.usd    Step 1  — UR16e URDF imported -> articulated arm (no gripper)
  ur16e_2f85_rigged.usd     Step 2  — arm + gripper rigidly attached (headless reference)
  ur16e_2f85_rigged_my.usd  Step 2  — GUI-attached variant, CAD solids preserved (4a input)
  gripper_2f85_tool.usd     pre-aligned gripper tool converted to USD
  UR16e_2F85_assembly.usd   full assembly USD (visual reference)
  ur16e_2f85_actuated.usd   Step 4a — legacy script output (scripts/rig_gripper_fingers.py)
```

## Regenerate
- Step 0–2 (CAD import, attach): Isaac Sim CAD/URDF importers — see
  `docs/UR16e_custom_gripper_tutorial.md` §3–§4. (Headless automation = template §8, TODO.)
- Step 4a actuation (legacy, 2F-85):
  ```bash
  PKG=/isaac-sim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311
  PYTHONPATH=$PKG LD_LIBRARY_PATH=$PKG/bin /isaac-sim/python.sh scripts/rig_gripper_fingers.py
  ```
- Template pipeline (config-driven, any end-effector): see `ee_template/README.md`
  (`build_ee.py` -> `ee_template/out/…`, then `verify_sim.py`).
