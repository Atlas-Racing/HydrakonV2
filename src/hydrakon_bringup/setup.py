from setuptools import find_packages, setup

package_name = 'hydrakon_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/zedx_bringup.py', 'launch/hydrakon_bringup.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Atlas Racing',
    maintainer_email='atlasracing@hw.ac.uk',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
