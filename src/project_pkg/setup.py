from setuptools import find_packages, setup

package_name = 'project_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sam',
    maintainer_email='sam@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_object_detector_node = project_pkg.object_detector:main',
            'object_tracker = project_pkg.object_tracker:main',
            'tracked_objects_visualizer = project_pkg.tracked_objects_visualizer:main',
            'fake_detection_publisher = project_pkg.fake_detection_publisher:main',
        ],
    },
)
