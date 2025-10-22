from setuptools import find_packages, setup
from glob import glob

package_name = "mapping"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/models", glob("share/models/*")),
    ],
    install_requires=["setuptools"],
    maintainer="samuel",
    maintainer_email="samuellimabraz@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mangalarga = mapping.mangalarga:main",
            "test_navigation = mapping.test.test_navigation:main",
            "test_detection = mapping.test.test_detection:main",
            "test_centralize = mapping.test.test_centralize:main",
        ],
    },
)