from setuptools import find_packages, setup

package_name = "gluon_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/yolo_detector.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="TODO@email.com",
    description="YOLO detector bridge for the Gluon robot arm vision approach pipeline.",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "yolo_detection_node = gluon_vision.yolo_detection_node:main",
        ],
    },
)
