from setuptools import find_packages, setup
import os
from glob import glob

package_name = "maze"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include models directory if it exists
        (
            os.path.join("share", package_name, "models"),
            glob("models/*") if os.path.exists("models") else [],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="samuel",
    maintainer_email="samuellimabraz@gmail.com",
    description="CBR 2025 Phase 4 - Autonomous Tello drone maze navigation with QR detection",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "mangalarga = maze.mangalarga:main",
            "nav = maze.nav:main",
            "qr = maze.qr:main",
            "waypoints = maze.waypoint_map:main",
        ],
    },
)
