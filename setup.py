from setuptools import find_packages, setup
from glob import glob
import os

package_name = '133finalproject'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),   # <- important
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='avni',
    maintainer_email='avni@todo.todo',
    description='133 final project package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)