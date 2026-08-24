from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drb3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='omver5669@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test = drb3.gripper_test:main',
            'force = drb3.force_test:main',
            'write = drb3.write_test:main',
            'brai = drb3.brill_test:main',
            'master = drb3.master_node:main',
            'writed = drb3.write_node:main',
            'braille = drb3.braille_node:main',
            'control = drb3.robot_control:main'

        ],
    },
)
