#!/usr/bin/env python3
"""Check metric depth rendering and its grayscale preview.

Run ``./tools/check_mujoco_depth.py`` from any directory. This script uses the
same interpreter selector as the simulation launchers; MUJOCO_PYTHON overrides
its choice. It needs a working OpenGL backend (a display, EGL, or OSMesa), even
though it does not open a viewer. It does not step or modify the saved models.
"""
from __future__ import annotations

import os
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]


def _use_launcher_python():
    marker = "CABINET_DEPTH_CHECK_LAUNCHED"
    if os.environ.get(marker) != str(Path(__file__).resolve()):
        environment = os.environ.copy()
        environment[marker] = str(Path(__file__).resolve())
        launcher = ROOT / "tools/mujoco_python.sh"
        os.execve(str(launcher), [str(launcher), str(Path(__file__).resolve())], environment)


def main():
    _use_launcher_python()
    # Keep this check from creating __pycache__ directories in the source tree.
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(ROOT / "src/total_cabinet_gazebo/mujoco"))
    sys.path.insert(0, str(ROOT / "src/rml63ii_eg2_4c2_gazebo/mujoco"))

    import numpy as np

    from camera_depth import depth_to_grayscale
    from environment import ArmEnv, mj
    from interaction_environment import TotalCabinetEnv

    width, height = 160, 120

    def check_depth(image):
        assert image.shape == (height, width), (image.shape, "expected H x W depth")
        assert image.dtype == np.float32, image.dtype
        assert np.isfinite(image).all() and (image > 0).all(), "invalid metric depth"

    def check_rgb(image):
        assert image.shape == (height, width, 3), (image.shape, "expected RGB")
        assert image.dtype == np.uint8, image.dtype

    def render(env, camera, **kwargs):
        return env.render(camera=camera, width=width, height=height, **kwargs)

    def check_environment(factory):
        with factory() as env:
            # Alternating modes at the same resolution exercises renderer reuse:
            # neither a preceding RGB frame nor a depth frame may leak its mode.
            rgb = render(env, "d435_rgb")
            check_rgb(rgb)
            depth = render(env, "d435_depth")
            check_depth(depth)
            check_rgb(render(env, "d435_depth", depth=False))
            depth_again = render(env, "d435_depth", depth=True)
            check_depth(depth_again)
            np.testing.assert_allclose(depth_again, depth, rtol=0, atol=1e-5)
            check_rgb(render(env, "d435_rgb"))
            check_depth(render(env, "d435_rgb", depth=True))
            np.testing.assert_allclose(render(env, "d435_depth"), depth, rtol=0, atol=1e-5)
            if factory is TotalCabinetEnv:
                global_cameras = [mj.mj_id2name(env.model, mj.mjtObj.mjOBJ_CAMERA, index)
                                  for index in range(env.model.ncam)]
                global_cameras = [name for name in global_cameras
                                  if name and name.startswith("global_")]
                assert global_cameras == ["global_default"], global_cameras
                check_rgb(render(env, "global_default"))
        print(f"PASS {factory.__name__}: automatic depth, explicit overrides, RGB/depth alternation")

    def check_known_distance():
        # The box front is z=-2 m. The camera looks down -Z. All visible pixels
        # must report optical-axis distance 2 m, including off-center pixels;
        # Euclidean ray length or normalized Z-buffer values would fail this.
        model = mj.MjModel.from_xml_string("""
        <mujoco>
          <statistic extent="1"/>
          <visual>
            <global offwidth="160" offheight="120"/>
            <map znear="0.01" zfar="10"/>
          </visual>
          <worldbody>
            <camera name="test_depth" pos="0 0 0" fovy="45"/>
            <geom type="box" pos="0 0 -2.1" size="5 5 0.1"
                  rgba="1 0 0 1"/>
          </worldbody>
        </mujoco>
        """)
        data = mj.MjData(model)
        mj.mj_forward(model, data)
        with mj.Renderer(model, height=height, width=width) as renderer:
            def frame(depth):
                if depth:
                    renderer.enable_depth_rendering()
                else:
                    renderer.disable_depth_rendering()
                renderer.update_scene(data, camera="test_depth")
                return renderer.render().copy()

            red_rgb = frame(False)
            red_depth = frame(True)
            check_depth(red_depth)
            np.testing.assert_allclose(red_depth, 2.0, rtol=0, atol=0.0002)
            model.geom_rgba[0] = (0, 0, 1, 1)
            blue_rgb = frame(False)
            blue_depth = frame(True)
            assert np.max(np.abs(red_rgb.astype(int) - blue_rgb.astype(int))) > 20, \
                "test surface color did not change"
            np.testing.assert_allclose(blue_depth, red_depth, rtol=0, atol=1e-5)
        print("PASS metric depth: 2 m plane, off-axis pixels, material-color independence")

    def check_grayscale():
        near, far = 0.1, 5.0
        invalid = np.array([[0, -1, np.nan, np.inf, -np.inf]], dtype=np.float32)
        output = depth_to_grayscale(invalid, near=near, far=far)
        assert output.shape == invalid.shape and output.dtype == np.uint8
        assert not output.any(), "invalid depths must be black"
        distances = np.array([[near / 2, near, (near + far) / 2, far, far * 20]],
                             dtype=np.float32)
        output = depth_to_grayscale(distances, near=near, far=far)
        assert output.shape == distances.shape and output.dtype == np.uint8
        assert output[0, 0] == output[0, 1] == 255, output
        assert abs(int(output[0, 2]) - 128) <= 1, output
        assert output[0, 3] == output[0, 4] == 0, output
        assert np.all(np.diff(output.astype(int), axis=1) <= 0), output
        for invalid_range in ((0, 5), (-1, 5), (5, 5), (6, 5), (0.1, np.inf),
                              (np.nan, 5)):
            try:
                depth_to_grayscale(distances, near=invalid_range[0], far=invalid_range[1])
            except ValueError:
                pass
            else:
                raise AssertionError(f"invalid display range accepted: {invalid_range}")
        try:
            depth_to_grayscale(np.zeros((2, 2, 3), dtype=np.uint8))
        except ValueError:
            pass
        else:
            raise AssertionError("RGB input must not be converted to a depth preview")
        print("PASS grayscale: fixed meter range, clipping, invalid depths, RGB rejection")

    print(f"Python: {sys.executable}\nMuJoCo: {mj.__version__}")
    check_grayscale()
    check_known_distance()
    for factory in (ArmEnv, TotalCabinetEnv):
        check_environment(factory)
    print("All depth rendering checks passed.")


if __name__ == "__main__":
    main()
