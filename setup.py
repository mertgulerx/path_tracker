from glob import glob
from setuptools import find_packages
from setuptools import setup


package_name = "path_tracker"


setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md", "LICENSE"]),
        (f"share/{package_name}/config", ["config/path_tracker.yaml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mertgulerx",
    maintainer_email="support.mertgulerx@gmail.com",
    description="Standalone ROS 2 Jazzy path tracker package for publishing traversed robot paths from TF.",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "path_tracker_node = path_tracker.path_tracker_node:main",
        ],
    },
)
