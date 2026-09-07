from glob import glob
from setuptools import setup

package_name = 'mycobot_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/boards', glob('boards/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='er',
    maintainer_email='ekinciorhandijvar@gmail.com',
    description='Hand-Auge-Kalibrierungswerkzeuge fuer myCobot 280 JN + D435i.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'charuco_detector = mycobot_calibration.charuco_detector:main',
            'mycobot_bridge = mycobot_calibration.joint_encoder_publisher:main',
            'joint_pose_gui = mycobot_calibration.joint_pose_gui:main',
            'estop_button = mycobot_calibration.estop_button:main',
        ],
    },
)
