from setuptools import find_packages, setup

package_name = 'rostoyo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marc Bestmann',
    maintainer_email='marc.bestmann@dlr.de',
    description='This ROS2 packet provides an interface to the existing project \
                 https://github.com/rabryan/pytuyo by Richard Bryan. A ROS2 node can output the \
                 measured value of a mitutoyo dial gauge, both as a topic and as a service.',
    license='MIT',
    tests_require=['pytest'],
    scripts=['scripts/rostoyo_client.py',
             'scripts/rostoyo_subscriber.py'],
    entry_points={
        'console_scripts': [
            'rostoyo_node = rostoyo.rostoyo_node:main'
        ],
    },
)
