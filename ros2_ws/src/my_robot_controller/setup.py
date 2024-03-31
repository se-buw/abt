from setuptools import find_packages, setup

package_name = 'my_robot_controller'

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
    maintainer='parallels',
    maintainer_email='adley.john.dsouza@uni-weimar.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'moveTurtle= my_robot_controller.moveTurtle:main',
        'turtle_publisher= my_robot_controller.turtle_publisher:main',
        'turtle_subscriber= my_robot_controller.turtle_subscriber:main',
        'advanced_turtle1 = my_robot_controller.advanced_turtle1:main',
        'advanced_turtle2 = my_robot_controller.advanced_turtle2:main',
        'advanced_turtle3 = my_robot_controller.advanced_turtle3:main',
        
        ],
    },
)
