from setuptools import setup

setup(
    name='gopigo3_ros',
    version='0.1.0',
    packages=['gopigo3_ros'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/gopigo3_ros']),
        ('share/gopigo3_ros', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'gopigo3_controller = gopigo3_ros.gopigo3_controller:main',
        ],
    },
)