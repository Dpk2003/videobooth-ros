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
                   'default_motion = doosan_motion_loop.default_motion:main',
                   'arc_motion = doosan_motion_loop.arc_motion:main',
                   'test= doosan_motion_loop.test:main',
                   'sine_wave= doosan_motion_loop.sine_wave:main',
                   'line_motion= doosan_motion_loop.line_motion:main',
        ],
    },
)
