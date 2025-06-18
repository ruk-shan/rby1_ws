from setuptools import find_packages, setup

package_name = 'point_cloud_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/models_config.yaml']),
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
            'point_cloud_pub = point_cloud_pub.point_cloud_pub:main',
            'model_visualizer = point_cloud_pub.model_visualizer:main',
            
        ],
    },
)
