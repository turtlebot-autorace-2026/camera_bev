from glob import glob
from setuptools import setup

package_name = 'camera_bev'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'README_KO.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/foxglove', glob('foxglove/*.json')),
        ('share/' + package_name + '/tools', glob('tools/*.py') + glob('tools/*.sh')),
    ],
    install_requires=['setuptools'], python_requires='>=3.8', zip_safe=True,
    maintainer='TurtleBot camera team', maintainer_email='maintainer@example.com',
    description='Direct calibrated raw-to-BEV ground projection with diagnostic topics',
    license='Apache-2.0', tests_require=['pytest'],
    entry_points={'console_scripts': ['bev_node = camera_bev.node:main']},
)
