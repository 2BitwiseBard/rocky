"""pip-installable core modules (NO ROS required — D009 layering).

    pip install -e .          # on the Pi, the laptop, or in CI

installs:
    pebble_gait, pebble_manip_adjacent   (from gait/)
    rocky_driver                          (from driver/rocky_driver/)

The ROS 2 packages in ros2/ import these; so do the bench scripts and the sim.
"""
from setuptools import setup

setup(
    name="pebble-core",
    version="0.6.0",
    description="Project ROCKY / Pebble core: gait engine + Feetech bus driver",
    py_modules=["pebble_gait", "pebble_manip_adjacent"],
    packages=["rocky_driver"],
    package_dir={"": "gait", "rocky_driver": "driver/rocky_driver"},
    install_requires=["numpy", "pyyaml"],
    extras_require={"hw": ["pyserial"], "sim": ["mujoco", "imageio"]},
    python_requires=">=3.10",
)
