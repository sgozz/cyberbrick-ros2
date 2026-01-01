from setuptools import setup

package_name = 'cyberbrick_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cyberbrick',
    maintainer_email='cyberbrick@example.com',
    description='CyberBrick ROS2 control nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vision_node = vision_node:main',
        ],
    },
)
