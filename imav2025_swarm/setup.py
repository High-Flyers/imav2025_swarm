from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'imav2025_swarm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "params"), glob("params/*.yaml"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hf',
    maintainer_email='kubal123.tomczak@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "position_publisher = imav2025_swarm.position_publisher:main",
            "swarm_control = imav2025_swarm.swarm_control:main"
        ],
    },
)
