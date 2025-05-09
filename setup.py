from setuptools import find_packages, setup

package_name = 'web_teleop_app_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/web_teleop_app_pkg/launch', ['launch/web_teleop_app.launch.py']),
        ('share/web_teleop_app_pkg/web_app', [
            'web_app/index.html',
            'web_app/joystick.js',
        ]),
        ('share/web_teleop_app_pkg/web_teleop_app_pkg', [
            'web_teleop_app_pkg/websocket_backend.py',
        ]),
    ],
    install_requires=['setuptools', 'rclpy', 'websockets'],
    zip_safe=True,
    maintainer='krissagar95',
    maintainer_email='Krishna_Sagar@artc.a-star.edu.sg',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
