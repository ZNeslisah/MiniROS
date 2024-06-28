from setuptools import setup
from glob import glob

package_name = 'ball_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='newans',
    maintainer_email='josh.newans@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	        'get_image = ball_tracker.get_image:main',
            'image_listener = ball_tracker.image_listener:main',
            'detect_aruco = ball_tracker.detect_aruco:main',
            'follow_aruco = ball_tracker.follow_aruco:main'

        ],
    },
)
