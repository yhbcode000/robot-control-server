from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install resource
        ('share/' + package_name + '/resource/physics_model', glob('resource/physics_model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shirox',
    maintainer_email='hobart.yang@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mujoco_node = sim.mujoco_node:main'
        ],
    },
)
