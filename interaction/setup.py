from setuptools import find_packages, setup
import os
from glob import glob

package_name = "interaction"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include launch files
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        ("share/" + package_name + "/models", glob("share/models/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lipedras",
    maintainer_email="lfljp@hotmail.com",
    description="Pacote para controle de drone por gestos para a CBR 2025.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gesture_recognizer = interaction.mav_gesture_recognizer:main",
            "test_detection = interaction.test.test_detection:main",
        ],
    },
)
