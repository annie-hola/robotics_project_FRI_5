from setuptools import setup

package_name = 'chase_avoid_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ciri',
    maintainer_email='you@example.com',
    description='Chase and avoid robot behavior controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [         
            'chase_avoid_robot = chase_avoid_robot.main:main',
            # script: package.file:function in file
        ],
    },
)
