from setuptools import find_packages, setup

package_name = 'semantic_mapper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
     ('share/' + package_name + '/config', ['config/data.yaml']),  
     ('share/' + package_name + '/weights', ['weights/best.pt']), 
     ('share/' + package_name + '/map', ['map/semantic_database.csv']), 
       ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pacman',
    maintainer_email='xyz@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        
            'semantic_mapper = semantic_mapper.semantic_database:main',
        ],
    },
)
