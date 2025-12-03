from setuptools import setup, find_packages

package_name = 'radar_detection_demo_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pandas'],
    zip_safe=True,
    maintainer='hanq',
    maintainer_email='hanq@todo.todo',
    description='Excel-based RadarDetection publisher & subscriber',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'excel_publisher = radar_detection_demo_py.excel_publisher:main',
            'radar_subscriber = radar_detection_demo_py.radar_subscriber:main',
            'udp2ros_bridge = radar_detection_demo_py.udp2ros_bridge:main',
        ],
    },
)

