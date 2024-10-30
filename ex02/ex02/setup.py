from setuptools import setup

package_name = 'ex02'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/turtlesim_with_carrot.launch.py']),
        ('share/' + package_name + '/config', ['config/carrot.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='email@mail.ru',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle1_broadcaster = ex02.turtle1_broadcaster:main',
            'turtle2_broadcaster = ex02.turtle2_broadcaster:main',
            'carrot_broadcaster = ex02.carrot_broadcaster:main',
            'turtle2_listener = ex02.turtle2_listener:main',
        ],
    },
)
