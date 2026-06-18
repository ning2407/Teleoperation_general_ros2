from glob import glob

from setuptools import find_packages, setup

package_name = 'teleoperation_general'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ning24',
    maintainer_email='ning24@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_manager = teleop_core.teleop_manager:main',
            'observation_publisher = teleop_core.observation_publisher:main',
            'keyboard_servo = teleop_hardware.keyboard_servo:main',
            'xbox_servo = teleop_hardware.xbox_servo:main',
            'phone_imu_servo = teleop_hardware.phone_imu_servo:main',
        ],
    },
)
