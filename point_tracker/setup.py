from setuptools import find_packages, setup

package_name = 'point_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/point_tracker_system.launch.py']),
        ('share/' + package_name + '/webapp', ['webapp/index.html']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='computeruse',
    maintainer_email='computeruse@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'point_publisher = point_tracker.point_publisher:main',
            'point_subscriber = point_tracker.point_subscriber:main'
        ],
    },
)
