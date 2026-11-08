from setuptools import setup

package_name = "dog_toy_finder_experiments"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/toy_search.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Dog Toy Finder Team",
    maintainer_email="dogtoyfinder@example.com",
    description="Full-system tests and evaluation for the Dog Toy Finder",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "toy_search_test = dog_toy_finder_experiments.toy_search_test:main",
            "evaluate_runs = dog_toy_finder_experiments.evaluate:main",
        ],
    },
)
