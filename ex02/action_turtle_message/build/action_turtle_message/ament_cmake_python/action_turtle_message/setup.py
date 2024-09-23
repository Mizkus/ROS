from setuptools import find_packages
from setuptools import setup

setup(
    name='action_turtle_message',
    version='0.0.0',
    packages=find_packages(
        include=('action_turtle_message', 'action_turtle_message.*')),
)
