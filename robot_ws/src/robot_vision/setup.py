from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/templates', glob('templates/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='mgt0@users.noreply.github.com',
    description='라인, 표지판, 손 인식',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'line_node = robot_vision.line_node:main',
            'sign_node = robot_vision.sign_node:main',
            'hand_node = robot_vision.hand_node:main',
            'hsv_tuner = robot_vision.hsv_tuner:main',
        ],
    },
)
