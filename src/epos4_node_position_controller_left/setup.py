from setuptools import find_packages, setup

package_name = 'epos4_node_position_controller_left'

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
    maintainer='marek',
    maintainer_email='maloisi@unibz.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'epos4_controller_left = epos4_node_position_controller_left.epos4_controller_left:main'
        ],
    },
)
