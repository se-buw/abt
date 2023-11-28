from setuptools import find_packages, setup

package_name = 'turtle_circle'

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
    maintainer='hadiya',
    maintainer_email='hadiya.kousar@uni-weimar.de',
    description='Turtle moving in circle',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "draw_circle = turtle_circle.draw_circle:main",
            "pose_subscriber = turtle_circle.pose_subscriber:main",
            "simple_bt = turtle_circle.simple_bt:main"
        ],
    },
)
