from setuptools import find_packages, setup

package_name = 'ex03'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/turtle.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kto-to',
    maintainer_email='email@mail.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle1_broadcaster = ex03.turtle1_broadcaster:main',
            'turtle2_broadcaster = ex03.turtle2_broadcaster:main',
            'turtle2_listener = ex03.turtle2_listener:main',
        ],
    },
)
