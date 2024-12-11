from setuptools import find_packages, setup
from glob import glob

package_name = 'pixar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Recursively include all .stl and .dae mesh files
        ('share/' + package_name + '/meshes', glob('meshes/**/*.stl', recursive=True) + glob('meshes/**/*.dae', recursive=True)),
        ('share/' + package_name + '/rviz',   glob('rviz/*')),
        ('share/' + package_name + '/urdf',   glob('urdf/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='The 133a Proj',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lamp_traj = pixar.lamp_traj_node:main',
            'momentum = pixar.momentum_test:main',
            'marker = pixar.marker_node:main',
        ],
    },
)
