import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'flight_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='High Flyers',
    maintainer_email='highflyers.polsl@gmail.com',
    description='Flight control package for IMAV 2025',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f"mission_control = {package_name}.mission_control_node:main"
        ],
    },
)
