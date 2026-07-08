from setuptools import find_packages, setup

package_name = 'hydrakon_planner'

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
    maintainer='Atlas Racing',
    maintainer_email='atlasracing@hw.ac.uk',
    description='Cone-corridor following controllers for Trackdrive/Autocross',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'corridor_controller_2d = hydrakon_planner.corridor_controller_2d:main',
            'corridor_controller_3d = hydrakon_planner.corridor_controller_3d:main',
        ],
    },
)
