from setuptools import setup

package_name = "dog_toy_finder_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/stereo.launch.py",
                                                "launch/vision.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Dog Toy Finder Team",
    maintainer_email="dogtoyfinder@example.com",
    description="Stereo vision for the Dog Toy Finder",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "stereo_camera_node = dog_toy_finder_vision.stereo_camera_node:main",
            "vision_node = dog_toy_finder_vision.vision_node:main",
        ],
    },
)
