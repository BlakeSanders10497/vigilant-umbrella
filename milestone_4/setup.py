from setuptools import setup

package_name = 'milestone_4'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marcusmlack',
    maintainer_email='marcusmlack@ufl.edu',
    description='Wall-following algorithm using a PID controller and LiDAR sensor.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_pid = milestone_4.sub_pub:main'
        ],
    },
)
