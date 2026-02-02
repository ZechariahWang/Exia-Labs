from setuptools import find_packages, setup

package_name = 'exia_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zechariah Wang',
    maintainer_email='zech@example.com',
    description='Navigation and control logic for Exia Ground robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dynamic_navigator_node = exia_control.dynamic_navigator_node:main',
            'ackermann_drive_node = exia_control.ackermann_drive_node:main',
        ],
    },
)
