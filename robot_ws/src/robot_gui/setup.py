from setuptools import find_packages, setup

package_name = 'robot_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='mgt0@users.noreply.github.com',
    description='ROS 기반 Qt GUI와 mode_manager',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mode_manager = robot_gui.mode_manager:main',
            'gui = robot_gui.gui:main',
        ],
    },
)
