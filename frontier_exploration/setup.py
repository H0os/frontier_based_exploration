from setuptools import find_packages, setup

package_name = 'frontier_exploration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aymon',
    maintainer_email='aymon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_utils = frontier_exploration.map_utils:main', 'exploration = frontier_exploration.exploration:main', 'training_data_collector = frontier_exploration.training_data_collector:main', 'metric_subscriber = frontier_exploration.metric_subscriber:main', 'tree_node = frontier_exploration.tree_node:main' , 'wavefront_frontier_exploration = frontier_exploration.wavefront_frontier_exploration:main'
        ],
    },
)
