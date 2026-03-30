from setuptools import find_packages, setup

package_name = 'systems'

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
    maintainer='Nico McMillan',
    maintainer_email='nico@todo.todo',
    description='State machine and systems watchdog for the farm robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_machine = systems.state_machine:main',
            'watchdog = systems.watchdog:main'
        ],
    },
)