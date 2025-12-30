from setuptools import setup

package_name = 'assignment1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Group',
    maintainer_email='your_email@example.com',
    description='Assignment package with Python ROS 2 nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detection = scripts.aruco_detection:main',
            'detection_primitives = scripts.detection_primitives:main'
            'mission_controller = scripts.mission_controller:main'
        ],
    },
)
