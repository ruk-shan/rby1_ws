from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rby1_visualizer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files (optional)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install mesh files!
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        # Install URDF files
        (os.path.join('share', package_name, 'rby1_urdf'), glob('rby1_urdf/*.urdf')),
        (os.path.join('share', package_name, 'rby1_urdf/meshes'), glob('rby1_urdf/meshes/*')),
        # Install launch files
        (os.path.join('share', package_name), glob(os.path.join(package_name, '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Publishes an STL marker',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stl_publisher = rby1_visualizer.stl_publisher_node:main',
            'fk_solver = rby1_visualizer.fk_solver:main',
        ],
    },
)

# package_name = 'rby1_visualizer'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='shan',
#     maintainer_email='ruk.shan@outlook.com',
#     description='TODO: Package description',
#     license='TODO: License declaration',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#         ],
#     },
# )
