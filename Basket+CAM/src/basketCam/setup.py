import os
from glob import glob
from setuptools import setup
from setuptools import find_packages, setup

package_name = 'basketCam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yang',
    maintainer_email='gkskaflxmf@naver.com',
    description='basket + cam',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'SegAnd3D = basketCam.SegAnd3D_node:main',
            'DepthCam = basketCam.DepthCam_node:main',
        ],
    },
)
