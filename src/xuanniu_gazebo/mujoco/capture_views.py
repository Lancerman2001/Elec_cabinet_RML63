#!/usr/bin/env python3
"""Render all three detents and four cameras from real MuJoCo state."""
import mujoco
from selector_backend import ROOT, SelectorSimulation, MODES
from image_utils import write_png


def main():
    sim=SelectorSimulation()
    with mujoco.Renderer(sim.model,height=1000,width=1000) as renderer:
        for mode,target in MODES.items():
            sim.command(target); sim.step(1500)
            assert sim.status().mode==mode
            for view in ('front','side','top','oblique'):
                renderer.update_scene(sim.data,camera=view)
                path=ROOT/'docs/screenshots/mujoco'/f'{view}_{mode.lower()}.png'
                write_png(path,renderer.render()); print(path)
    sim.reset()


if __name__=='__main__': main()
