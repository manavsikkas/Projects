import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'demo2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][xma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('rviz', '*.rviz')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = demo2.my_node:main',
            'my_node2 = demo2.my_node2:main',
            'my_node3 = demo2.my_node3:main',
        ],
    },
)
