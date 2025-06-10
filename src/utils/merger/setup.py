from setuptools import find_packages, setup

package_name = 'merger'

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
    maintainer='lamp',
    maintainer_email='salamp@salamp.id',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom = merger.odom_tf_rebroadcaster:main',
            'map = merger.map_rebroadcaster:main',
            'limit = merger.lidar_limiter:main',
            'watcher = merger.watcher:main',
        ],
    },
)
