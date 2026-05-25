import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'hybrid_astar_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vatcraft',
    maintainer_email='23020783@vnu.edu.vn',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'planner_node      = hybrid_astar_planner.planner_node:main',
            'pure_pursuit      = hybrid_astar_planner.pure_pursuit:main',
            'teb_controller = hybrid_astar_planner.teb_controller:main',
        ],
    },
)
