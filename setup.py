from setuptools import setup

setup(
    name='ros1_fuzzer',
    version='1.0.0',
    packages=['ros1_fuzzer'],
    url='https://www.aliasrobotics.com',
    license='GPLv2',
    author='Alias Robotics',
    author_email='contact@aliasrobotics.com',
    description='A ROS subscriber fuzzing tool for ROS systems',
    keywords=['network', 'fuzzing', 'ros'],
    entry_points={
        'console_scripts': ['ros1_fuzzer=ros1_fuzzer.ros_fuzzer:main'],
    },
    install_requires=[
        'hypothesis==3.82',
        'attrs==19.1.0',
        'enum34==1.1.6',
        'psutil==5.6.2'
    ],
    tests_requires=[
        'pytest==4.5.0',
    ],
    include_package_data=True,
)
