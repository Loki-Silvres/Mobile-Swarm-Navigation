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
     ('share/' + package_name + '/config', ['config/data.yaml']),  # Add your config files here
     ('share/' + package_name + '/config', ['config/inter-iit-final4.pt']),  # Add your config files here
       ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pacman',
    maintainer_email='guptakanishk217@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        
            'semantic_mapper = semantic_mapper.semantic_database:main',
        ],
    },
)
