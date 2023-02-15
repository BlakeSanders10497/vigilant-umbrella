from setuptools import setup
import os
from glob import glob

package_name = 'project_1'

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*_launch.py")),
        ("share/" + package_name + "/data", glob("data/*.txt")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="marcusmlack",
    maintainer_email="marcusmlack@ufl.edu",
    description="TODO: Package description",
    license="TODO: License description",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            'run_joy = project_1.joy_test:main'
        ],
    },
)