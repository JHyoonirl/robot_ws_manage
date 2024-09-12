
#!/usr/bin/env python3
import glob
import os

from setuptools import find_packages, setup

package_name = 'manage_hardware'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (share_dir + '/launch', glob.glob(os.path.join('launch','*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='irl',
    maintainer_email='irl@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={ 
        'console_scripts': [
            'sensor_data = manage_hardware.sensor_data:main',
            'sensor_graph = manage_hardware.sensor_graph:main',
            'visual_data = manage_hardware.sensor_graph:main',
            'rasp_pi_pwm = manage_hardware.rasp_pi_pwm:main',
            'pwm_ve = manage_hardware.pwm_ve:main',
            'data_load = manage_hardware.data_load:main',
            'data_save = manage_hardware.data_save:main',
        ],
    },
)
