from setuptools import setup

package_name = 'doosan_motion_loop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Doosan A0509 continuous motion loop',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
                   'spline_motion = doosan_motion_loop.spline_motion:main',
                   'camera_trigger = doosan_motion_loop.camera_trigger:main',
                   'arc_motion = doosan_motion_loop.arc_motion:main',
                   'arc_motion1 = doosan_motion_loop.arc_motion1:main',
                   'arc_motion2= doosan_motion_loop.arc_motion2:main',
        ],
    },
)
