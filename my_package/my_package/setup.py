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
    maintainer_email='pnum4095@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_package.my_node:main',
            'talker = my_package.publisher_member_function:main',
            'listener = my_package.subscriber_member_function:main',
            'communication = my_package.communication:main',
            'odom_node = my_package.odom_node:main',
            'path_pub_node = my_package.path_pub_node:main',
            'path_tracking = my_package.path_tracking:main',
            'make_path = my_package.make_path:main',
        ],
    },
)
