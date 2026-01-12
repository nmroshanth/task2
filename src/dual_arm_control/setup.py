from setuptools import find_packages, setup

package_name = 'dual_arm_control'

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
    maintainer='nmroshanth',
    maintainer_email='nmrosh2811@gmail.com',
    description='Dual-arm Cartesian Jacobian position controller',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cartesian_jacobian_position_control = dual_arm_control.cartesian_jacobian_position_control:main',
        ],
    },
)