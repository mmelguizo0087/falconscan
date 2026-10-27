from glob import glob

from setuptools import find_packages, setup

package_name = "falconscan_volume"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Cainel S.A.S.",
    maintainer_email="dev@cainel.co",
    description="Reconstrucción y volumen de volquetas con el LiDAR transversal (Falcon Scan)",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "volume_node = falconscan_volume.volume_node:main",
            "sim_portico = falconscan_volume.sim_portico:main",
        ],
    },
)
