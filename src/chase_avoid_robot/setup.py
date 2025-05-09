from setuptools import find_packages, setup

package_name = 'chase_avoid_robot'

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
    maintainer='ubuntu',
    maintainer_email='anhanhanh151199@gmail.com',
    description='Chase Avoid Robot Controller',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [         
            'chase_avoid_robot = chase_avoid_robot.main:main',
            # script: package.file:function in file
        ],
    },
)
