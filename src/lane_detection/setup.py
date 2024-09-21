from setuptools import find_packages, setup
from glob import glob

package_name = 'lane_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: ['yolov5/best.pt'],
        package_name: ['yolov5/img.jpg'],

    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*launch.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hiep',
    maintainer_email='hiepdoan7820@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_lane = lane_detection.yolov5.detect_lane:main',
            'detect_sign = lane_detection.yolov5.detect_sign:main',
            'process_image = lane_detection.yolov5.process_image:main',
            'detect = lane_detection.yolov5.detect:main'
        ],
    },
)
