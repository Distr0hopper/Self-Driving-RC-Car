from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'kyoto'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seb',
    maintainer_email='sebastian.bylehn@gmail.com',
    description='Kyoto UGV control and visualization package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui = kyoto.gui:main',
            'motor_control = kyoto.motor_control:main',
            'xbox_control = kyoto.xbox_control:main',
            'apriltag_detector = kyoto.apriltag_detector:main',
            'plotter = kyoto.plotter:main',
            'guikeypwm = kyoto.gui_key_pwm:main',
            'velocity = kyoto.velocity:main',

        ],
    },
)