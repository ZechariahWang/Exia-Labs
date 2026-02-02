from setuptools import find_packages, setup

package_name = 'exia_driver'

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
    description='Hardware drivers for Exia Ground robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rc_driver_node = exia_driver.rc_driver_node:main',
        ],
    },
)
