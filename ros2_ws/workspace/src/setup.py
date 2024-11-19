from setuptools import setup

package_name = 'mob_rob_obstacle_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ponti.luca00@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_obstacle = mob_rob_obstacle_avoidance.move_obstacle:main',
            'control_robot = mob_rob_obstacle_avoidance.control_robot:main',
            'move_circle = mob_rob_obstacle_avoidance.move_circle:main',
            'move_rectangle = mob_rob_obstacle_avoidance.move_rectangular:main',
            'movement = mob_rob_obstacle_avoidance.movementHardcoded:main'
        ],
    },
)
