from setuptools import find_packages, setup

package_name = 'manage_hardware'

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
    maintainer='irl',
    maintainer_email='irl@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_data = manage_hardware.sensor_data:main',
            'visual_data = manage_hardware.sensor_graph:main',
            'rasp_pi_pwm = manage_hardware.rasp_pi_pwm:main',
        ],
    },
)
