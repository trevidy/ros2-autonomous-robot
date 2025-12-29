from setuptools import find_packages, setup

package_name = 'autonomous_turtle'

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
    maintainer='trevidy',
    maintainer_email='trevidy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_turtle = autonomous_turtle.move_turtle:main', # Entry point for the move_turtle node
            'avoid_walls = autonomous_turtle.avoid_walls:main', # Entry point for the avoid_walls node
        ],
    },
)
