#!/usr/bin/env python3
"""Rebuild the arm and full cabinet from the sources inside this workspace."""
import os
from pathlib import Path
import runpy
import sys

source = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(source / "rml63ii_eg2_4c2_gazebo/mujoco"))
sys.path.insert(0, str(source / "total_cabinet_gazebo/scripts"))
# Existing nameplate textures are part of the model. Reuse them by default so
# regeneration does not require a Windows font installation. Set to 0 when
# changing plate text/dimensions, and supply CABINET_NAMEPLATE_FONT if needed.
os.environ.setdefault("CABINET_REUSE_NAMEPLATES", "1")
for relative in (
    "IO_gazebo/mujoco/generate_mjcf.py",
    "fugui_gazebo/mujoco/generate_mjcf.py",
    "rml63ii_eg2_4c2_gazebo/scripts/urdf_to_mjcf.py",
    "total_cabinet_gazebo/scripts/build_cabinet.py",
    "total_cabinet_gazebo/scripts/build_interaction.py",
):
    path = source / relative
    sys.path.insert(0, str(path.parent))
    sys.argv = [str(path)]
    print(f"Rebuilding {relative}", flush=True)
    try:
        runpy.run_path(str(path), run_name="__main__")
    except SystemExit as error:
        if error.code not in (None, 0):
            raise

