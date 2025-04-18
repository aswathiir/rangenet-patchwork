from setuptools import setup

package_name = 'rangenet_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aswathi',
    maintainer_email='aswathiranjikk@gmail.com',
    description='ROS 2 wrapper for RangeNet++',
    license='MIT',
    entry_points={
        'console_scripts': [
            'node = rangenet_ros2.node:main',
            
        ],
    },
)

