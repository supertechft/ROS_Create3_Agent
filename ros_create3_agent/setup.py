from setuptools import find_packages, setup
import os
from glob import glob

package_name = "ros_create3_agent"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")), # Launch Files
        (os.path.join("share", package_name), [".env"]),  # .env file
        # Web templates and static files
        (os.path.join("share", package_name, "web/templates"), glob("ros_create3_agent/ros_create3_agent/web/templates/*")),
        (os.path.join("share", package_name, "web/static/css"), glob("ros_create3_agent/ros_create3_agent/web/static/css/*")),
        (os.path.join("share", package_name, "web/static/js"), glob("ros_create3_agent/ros_create3_agent/web/static/js/*")),
    ],
    install_requires=[
        "setuptools",
        "rclpy",
        "jpl-rosa @ git+https://github.com/supertechft/ROSA.git@ASR#egg=jpl-rosa",
        "flask",
    ],
    zip_safe=True,
    maintainer="Prince Singh",
    maintainer_email="psingh@supertechft.org",
    description="Embodied ROS agent for the iRobot Create 3 robot",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "agent = ros_create3_agent.agent:main",
        ],
    },
    python_requires=">=3.8",
    classifiers=[
        "Programming Language :: Python",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: OS Independent",
    ],
)
