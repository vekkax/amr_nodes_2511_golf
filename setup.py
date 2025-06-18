from setuptools import find_packages, setup

package_name = 'amr_nodes_2511_golf'

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
    maintainer='vekkaz',
    maintainer_email='santiagov3lasquez@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                        'time_to_collision_node = amr_nodes_2511_golf.time_to_collission_node:main',
                        'fscan_node = amr_nodes_2511_golf.fscan_node:main',
                        'rumble_node = amr_nodes_2511_golf.rumble_node:main',
                        'vel_stamper_node = amr_nodes_2511_golf.vel_stamper_node:main',
                        'wall_following_node = amr_nodes_2511_golf.wall_following_node:main',
        ],
    },
)
