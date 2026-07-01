# CAD sources (not committed)

These are large vendor CAD files (100s of MB) excluded from git (`.gitignore`).
Obtain them from the vendors and drop them here to run the STEP→USD steps.

```
cad/
  ur16e/UR16e.step                     UR16e arm CAD (Universal Robots official STEP)
  grippers/2f-85-robotiq/Step/2F85_Opened_20190924.STEP   Robotiq 2F-85 gripper
  grippers/GRP-CPL-062.STEP            Robotiq UR coupling (ISO 9409-1-50-4-M6)
  gripper_2f85_tool.step               pre-aligned coupling+gripper tool (mount=origin, +Z=approach)
  assemblies/UR16e_2F85_assembly.step  full arm+coupling+gripper assembly (visual reference)
  assemblies/UR16e_with_3finger_gripper.step   3-finger variant (unused yet)
```

Sources:
- UR16e arm: Universal Robots download center (support.universal-robots.com).
- Robotiq 2F-85 / coupling: Robotiq support site.
- `gripper_2f85_tool.step` is a *derived* asset (coupling+gripper pre-aligned to the
  mount-face=origin, +Z=approach convention) built with OpenCASCADE — regenerate per the
  tutorial's Step-2 convention if missing.

> NOTE: two source dirs (`2f-85-grippers-robotiq/`, `UR16e.step/`) currently sit at the repo
> root instead of under `cad/` because they are owned by a different user and could not be
> relocated without sudo. They are gitignored in both locations. To tidy on disk:
> `sudo mv UR16e.step/UR16e.step cad/ur16e/ && sudo mv 2f-85-grippers-robotiq cad/grippers/2f-85-robotiq`
