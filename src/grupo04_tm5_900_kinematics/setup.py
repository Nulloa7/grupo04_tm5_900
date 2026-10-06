from setuptools import find_packages, setup

package_name = 'grupo04_tm5_900_kinematics'

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
    maintainer='nico',
    maintainer_email='nico@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'fk_node = grupo04_tm5_900_kinematics.fk_node:main',
        'ik_node = grupo04_tm5_900_kinematics.ik_node:main',
        'ik_node_pinv = grupo04_tm5_900_kinematics.ik_node_pinv:main',
    ],
   },
)
