from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'isaac_ml'

def get_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
    ]
    
    # USD 파일들을 재귀적으로 포함
    usd_files = []
    for root, _, files in os.walk('usd'):
        if files:
            dest = os.path.join('share', package_name, root)
            sources = [os.path.join(root, f) for f in files]
            data_files.append((dest, sources))
            
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=get_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'start_sim = isaac_ml.scripts.start_sim:main',
            'four_ws_control = isaac_ml.scripts.four_ws_control:main',
            'four_ws_control_pos = isaac_ml.scripts.four_ws_control_pos:main',
            'keyboard_teleop = isaac_ml.scripts.keyboard_teleop:main',
            'state_machine = isaac_ml.scripts.state_machine:main',
            'drone_local_path = isaac_ml.scripts.drone_local_path:main',
        ],
    },
) 