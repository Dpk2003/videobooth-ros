from setuptools import find_packages, setup

package_name = 'whatsapp_robot'

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
    maintainer='deepak',
    maintainer_email='deepaks799420@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_motion_server = whatsapp_robot.robot_motion_server:main',
            'camera_trigger = doosan_motion_loop.camera_trigger:main',
            'tracking_shot = doosan_motion_loop.tracking_shot_arc_motion:main',
        ],
    },
)
