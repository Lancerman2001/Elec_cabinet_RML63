#!/usr/bin/env python3
"""Start the original cabinet demo with only local runtime paths."""
from pathlib import Path
import runpy
import sys

workspace = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(workspace / "src/rml63ii_eg2_4c2_gazebo/mujoco"))
runtime = workspace / "src/total_cabinet_gazebo/mujoco"
sys.path.insert(0, str(runtime))
runpy.run_path(str(runtime / "cabinet_gui.py"), run_name="__main__")
