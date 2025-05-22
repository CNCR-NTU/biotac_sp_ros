from setuptools import find_packages, setup

package_name = 'mpu9250_orientation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pedro Machado',
    maintainer_email='pedro.machado@ntu.ac.uk',
    description='MPU9250 ROS2 package',
    license='LGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orientation_publisher = mpu9250_orientation.orientation_publisher:main',
            'plot_orientation = mpu9250_orientation.plot_orientation:main',
        ],
    },
)
