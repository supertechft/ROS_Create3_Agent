from setuptools import find_packages, setup
import os
from glob import glob

package_name = "rosa_create3_agent"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name), [".env"]),  # Include .env file in package
    ],
    install_requires=[
        "setuptools",
        "rclpy",
        "ament-index-python",
        "jpl-rosa",
    ],
    zip_safe=True,
    maintainer="Prince Singh",
    maintainer_email="psingh@supertechft.org",
    description="Embodied ROSA agent for the iRobot Create 3 robot",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "agent = rosa_create3_agent.rosa_create3_agent:main",
        ],
    },
    python_requires=">=3.8",
    classifiers=[
        "Programming Language :: Python",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: OS Independent",
    ],
)
