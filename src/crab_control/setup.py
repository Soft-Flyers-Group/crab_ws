from setuptools import find_packages, setup
from glob import glob
package_name = 'crab_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')
        ),
        ('share/'+ package_name + '/config',
        glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='odinroast',
    maintainer_email='ruthvick2005@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'servo_node = crab_control.servo_node:main',
            'servo_controller = crab_control.servo_controller:main',
            'load_cell_node = crab_control.load_cell_node:main',
            'imu_node = crab_control.imu_node:main',
            'camera_node = crab_control.camera_node:main',
        ],
    },
)
