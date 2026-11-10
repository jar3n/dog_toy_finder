from setuptools import setup

package_name = "dog_toy_finder_motion"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Dog Toy Finder Team",
    maintainer_email="dogtoyfinder@example.com",
    description="Closed-loop motion primitives for the Dog Toy Finder",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mover_test = dog_toy_finder_motion.move_test:main",
            "manual_drive = dog_toy_finder_motion.manual_drive:main",
            "cmd_vel_mux = dog_toy_finder_motion.cmd_vel_mux:main",
        ],
    },
)
