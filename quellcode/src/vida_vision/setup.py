from glob import glob

from setuptools import setup

package_name = 'vida_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/weights', glob('weights/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='er',
    maintainer_email='ekinciorhandijvar@gmail.com',
    description='YOLO11-seg Schraubenerkennung + 3D-Zielerzeugung (myCobot 280 JN + D435i).',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'vida_detector = vida_vision.vida_detector_node:main',
        ],
    },
)
