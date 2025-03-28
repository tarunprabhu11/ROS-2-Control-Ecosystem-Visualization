from setuptools import find_packages, setup

package_name = 'node_system'

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
    maintainer='Tarun Prabhu',
    maintainer_email='tarun.prabhu11@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_a = node_system.node_a:main',
            'node_b = node_system.node_b:main',
            'node_c = node_system.node_c:main',
            'node_d = node_system.node_d:main',
        ],
    },
)
