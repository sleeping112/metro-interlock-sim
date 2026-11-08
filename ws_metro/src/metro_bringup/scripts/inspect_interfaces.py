"""Print the public interface of the running simulation.

Usage: ros2 run metro_bringup inspect_interfaces.py
"""

import subprocess
import sys


def run(command):
    print(f"\n$ {command}")
    result = subprocess.run(command, shell=True, check=False, text=True)
    if result.returncode != 0:
        print(f"[exit {result.returncode}]")


def main():
    for command in (
        "ros2 topic list -t",
        "ros2 service list -t",
        "ros2 node list",
    ):
        run(command)
    return 0


if __name__ == "__main__":
    sys.exit(main())
