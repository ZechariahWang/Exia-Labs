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
    install_requires=[
        'setuptools',
        'httpx>=0.27',
        'tomli-w>=1.0',
        'tomli>=2.0',
    ],
    zip_safe=True,
    maintainer='Zechariah Wang',
    maintainer_email='zechariahwang@gmail.com',
    description='Navigation and control logic for Exia Ground robot probably.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dynamic_navigator_node = exia_control.dynamic_navigator_node:main',
            'ackermann_drive_node = exia_control.ackermann_drive_node:main',
            'nav_to = exia_control.nav_to_cmd:main',
            'teleop = exia_control.teleop_node:main',
            'hw_teleop = exia_control.hw_teleop_node:main',
            'lss_calibrate = exia_control.lss_calibrate:main',
        ],
    },
)
