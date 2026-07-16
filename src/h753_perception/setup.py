from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'h753_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jyl1015',
    maintainer_email='jyl1015@example.com',
    description='YOLO RGB-D person and blue-clothing perception for H753 modes.',
    license='Apache-2.0',
    tests_require=['pytest'],
)
