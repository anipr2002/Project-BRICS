from setuptools import find_packages, setup

package_name = 'brics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','ur_robot_driver'],
    zip_safe=True,
    maintainer='anipr2002',
    maintainer_email='anipr2002@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'minimal_publisher = brics.publisher:main',
            'minimal_subscriber = brics.subscriber:main',
        ],
    },
)
