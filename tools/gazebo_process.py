#!/usr/bin/python3
"""Own one launch process group and stop only its isolated Gazebo master."""
import os
from pathlib import Path
import signal
import subprocess
import sys


def owned_servers(session):
    servers = []
    for process in Path("/proc").iterdir():
        if not process.name.isdecimal():
            continue
        try:
            if os.getsid(int(process.name)) == session and (process / "comm").read_text().strip() == "gzserver":
                servers.append(int(process.name))
        except (OSError, ProcessLookupError):
            continue
    return servers


def main():
    stopping = False

    def request_stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    arguments = sys.argv[1:]
    stop_environment = os.environ.copy()
    for argument in arguments:
        if argument.startswith("master_uri:="):
            stop_environment["GAZEBO_MASTER_URI"] = argument.split(":=", 1)[1]
    child = subprocess.Popen(["ros2", "launch", "total_cabinet_gazebo", "interaction.launch.py", *arguments],
                             start_new_session=True)
    try:
        while not stopping:
            try:
                return child.wait(timeout=.2)
            except subprocess.TimeoutExpired:
                pass
        # Only publish a shutdown request when our own session has a server;
        # a launch that failed its occupied-port check owns no Gazebo process.
        servers = owned_servers(child.pid)
        if child.poll() is None and servers:
            stop = Path(__file__).resolve().parents[1] / "install/lib/rml63ii_eg2_4c2_gazebo/gazebo_stop"
            try:
                subprocess.run([str(stop)], env=stop_environment, timeout=8, check=True)
                child.wait(timeout=5)
                return 0
            except (subprocess.TimeoutExpired, subprocess.CalledProcessError, OSError):
                pass
            # Some Gazebo Classic / ROS 2 control plugin versions deadlock
            # during model destruction. Bound that wait and reclaim only the
            # server PIDs in our own session, leaving other simulations alone.
            remaining = owned_servers(child.pid)
            if remaining:
                print("Gazebo shutdown timed out; reclaiming this launch's server.", flush=True)
                for process_id in remaining:
                    try:
                        os.kill(process_id, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                try:
                    child.wait(timeout=10)
                    return 0
                except subprocess.TimeoutExpired:
                    pass
        if child.poll() is None:
            try:
                os.kill(child.pid, signal.SIGINT)
                child.wait(timeout=10)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass
        return 0
    finally:
        if child.poll() is None:
            try:
                os.killpg(child.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    print("Timed out waiting for the stopped launch process to be reaped.", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
