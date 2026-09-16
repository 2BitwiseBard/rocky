from setuptools import setup

package_name = "rocky_gait"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tyler Bensko",
    maintainer_email="tbensko@gmail.com",
    description="Wave-gait engine node for Pebble (wraps pebble_gait)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "gait_node = rocky_gait.gait_node:main",
        ],
    },
)
