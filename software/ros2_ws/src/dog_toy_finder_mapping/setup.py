from setuptools import setup

package_name = "dog_toy_finder_mapping"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_data={package_name: ["static/*", "web/*"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/mapping.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Dog Toy Finder Team",
    maintainer_email="dogtoyfinder@example.com",
    description="Occupancy mapping for the Dog Toy Finder",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "occupancy_mapper = dog_toy_finder_mapping.occupancy_mapper:main",
            "map_webserver = dog_toy_finder_mapping.map_webserver:main",
        ],
    },
)
