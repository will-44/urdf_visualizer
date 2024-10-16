from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'urdf_visualizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.*'))),
        (os.path.join('share', package_name, 'urdf/accessories'), glob('urdf/accessories/*')),
        (os.path.join('share', package_name, 'urdf/config'), glob('urdf/config/*')),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.*'))),
        (os.path.join('share', package_name, 'meshes/m1013_blue'), glob('meshes/m1013_blue/*')),
        (os.path.join('share', package_name, 'meshes/m1013_collision'), glob('meshes/m1013_collision/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='will',
    maintainer_email='guillaume.dupoiron@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
