import os
from setuptools import setup
from glob import glob

package_name = 'best_vertices'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chathushka-sutd',
    maintainer_email='chathushka.ranasinghe41@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['exec_py = best_vertices.best_vertices_node:main',
        ],
    },
)
