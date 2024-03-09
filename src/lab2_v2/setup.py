from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lab2_v2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Piotr Patek, Damian Baraniak',
    maintainer_email='piotr.patek.stud@pw.edu.pl damian.baraniak.stud@pw.edu.pl',
    description='ROS2 node for turtle control. Made for ANRO subject on WUT',
    license='BSD-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'anro_turtle = lab2_v2.anro_turtle:main'
        ],
    },
)
