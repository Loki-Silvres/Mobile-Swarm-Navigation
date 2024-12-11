from setuptools import find_packages
from setuptools import setup

setup(
    name='tortoisebot_firmware',
    version='0.0.0',
    packages=find_packages(
        include=('tortoisebot_firmware', 'tortoisebot_firmware.*')),
)
