from setuptools import find_packages, setup

package_name = 'my_robot_wheel'

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
    maintainer='yuheng',
    maintainer_email='yuheng272002@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "wheel_node = my_robot_wheel.wheel_node:main",
            "static_state_publisher = my_robot_wheel.static_state_publisher:main",
        ],
    },
)
