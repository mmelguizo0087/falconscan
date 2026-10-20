from glob import glob

from setuptools import find_packages, setup

package_name = "falconscan_velocity"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Cainel S.A.S.",
    maintainer_email="dev@cainel.co",
    description="Estimación de velocidad de volquetas por correlación de perfiles LiDAR",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "velocity_node = falconscan_velocity.velocity_node:main",
            "sim_publisher = falconscan_velocity.sim_publisher:main",
        ],
    },
)
