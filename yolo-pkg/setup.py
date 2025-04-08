from setuptools import find_packages, setup

package_name = 'yolo'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'cv_bridge',
        'rclpy',
        'numpy',
        'ultralytics'
    ],
    zip_safe=True,
    maintainer='ohin',
    maintainer_email='ohin.kyuu@gmail.com',
    description='YOLO human detection',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ros_main = yolo.ros_main:main',
            'yolo_color = yolo.yolo_color:main',
            'mp42bag = yolo.mp42bag:main',
        ],
    },
)
