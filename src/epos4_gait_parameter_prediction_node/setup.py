from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'epos4_gait_parameter_prediction_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    
    (os.path.join('share', package_name, 'data'),
        glob('epos4_gait_parameter_prediction_node/data/*.pkl')),
    (os.path.join('share', package_name, 'lstm_models'),
        glob('epos4_gait_parameter_prediction_node/lstm_models/*.pth')),
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
            'epos4_gait_parameter_prediction_node = epos4_gait_parameter_prediction_node.epos4_gait_parameter_prediction_node:main'
        ],
    },
)
