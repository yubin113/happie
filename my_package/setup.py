from setuptools import setup

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SSAFY',
    maintainer_email='teayoung0812@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_package.my_node:main',
            'talker = my_package.publisher_member_function:main',
            'listener = my_package.subscriber_member_function:main',
            'communication=my_package.communication:main',
            'odometry = my_package.odometry_node:main',
            'lidar_subscriber = my_package.lidar_subscriber:main',
            'image_parser = my_package.perception:main',
            'hand_control = my_package.handcontrol:main',
            'make_path = my_package.make_path:main',
            'path_tracking = my_package.path_tracking:main',
            'path_pub = my_package.path_pub:main'
        ],
    },
)
