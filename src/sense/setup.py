from setuptools import find_packages, setup

package_name = 'sense'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            # 'motor_feedback_node = sense.motor_feedback_node:main',
            'keyboard_node = sense.keyboard_node:main',
            # 'depth_camera_node = sense.depth_camera_node:main',
            'microphone_node = sense.microphone_node:main',
            'text_input_node = sense.text_input_node:main'
        ],
    },
)
