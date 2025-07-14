from setuptools import find_packages, setup

package_name = 'kinematics_solver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shan',
    maintainer_email='ruk.shan@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'transform_listener = kinematics_solver.transform_listener_node:main',
            'camera_to_torso_transform = kinematics_solver.camera_to_torso_transform:main',
            'fk_solver = kinematics_solver.fk_solver_03:main',
        ],
    },
)
