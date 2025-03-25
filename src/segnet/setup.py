from setuptools import find_packages, setup

package_name = 'segnet'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/segnet.launch.py']),
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
            'yoloseg = segnet.yolo_segnet:main', # YOLO Segmentation
            'yolodst = segnet.yolo_denset:main', # YOLO Detection and Segmentation
            'jetdet = segnet.jetson_denet:main', # Jetson Inference DetectNet
            'jetseg = segnet.jetson_segnet:main', # Jetson Inference SegNet
            'denset = segnet.denset:main', # Jetson DetectNet and SegNet
        ],
    },
)