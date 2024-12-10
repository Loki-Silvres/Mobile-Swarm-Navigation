from setuptools import find_packages, setup

package_name = 'scheduler'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
             ('share/' + package_name + '/launch', ['launch/scheduler_launch.py']),  # Add your config files here
   
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pacman',
    maintainer_email='your_name@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'scheduler_2 = scheduler.scheduler_2:main',
        	'sub_scheduler = scheduler.sub_scheduler:main',
            'scheduler_3 = scheduler.scheduler_3:main',
        	
        	
        ],
    },
)
