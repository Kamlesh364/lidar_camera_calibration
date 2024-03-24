from setuptools import find_packages, setup
from os.path import join
from glob import glob

package_name = 'lidar_camera_calibration'
submodules = 'lidar_camera_calibration/utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kamlesh',
    maintainer_email='kkamlesh.p47@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_node = lidar_camera_calibration.calibration:main',
            'calibration_projection_node = lidar_camera_calibration.calibration_projection_node:main',
        ],
    },
)
