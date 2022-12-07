from setuptools import setup
from setuptools import find_packages

package_name = 'simcharts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Simon Lexau',
    maintainer_email='simon.lexau@ntnu.no',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simcharts = simcharts.launch_simcharts:main',
            'local_traffic_node = simcharts.launch_local_traffic_node:main',
            'dev_test = simcharts.devTest:main'
        ],
    },
)
