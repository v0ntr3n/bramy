from setuptools import find_packages, setup

package_name = 'bramy'

setup(
    name=package_name,
    version='0.19.5',
    packages=[package_name, f'{package_name}.camera', f'{package_name}.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+package_name+'/launch', ['launch/bramy.launch.py']),
    ],
    package_data={
        '': ['libs/*.so'],  # this includes .so in install
    },
    install_requires=['setuptools'],
    zip_safe=True,
    author='Mikael Arguedas',
    author_email='mikael@osrfoundation.org',
    maintainer='Aditya Pande, Alejandro Hernandez Cordero',
    maintainer_email='aditya.pande@openrobotics.org, alejandro@openrobotics.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples of minimal subscribers using rclpy.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = bramy.camera_publisher:main',
            'get_control = bramy.get_control:main',
            'tracking = bramy.tracking:main',
        ],
    },
)
