from setuptools import find_packages, setup

package_name = 'robot_base'

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
    description='모터 구동, 오도메트리',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'base_node = robot_base.base_node:main',
        ],
    },
)
