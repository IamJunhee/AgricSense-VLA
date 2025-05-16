from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gemma_ros_client'

data_files_list = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob(os.path.join('launch', '*launch.[pxy]*'))),
    ('share/' + package_name + '/config', glob(os.path.join('config', '*.yaml'))),
]

for root, dirs, files in os.walk('resource'):
    if files:
        destination_folder = os.path.join('share', package_name, root)
        source_files_in_dir = [os.path.join(root, f) for f in files]
        data_files_list.append((destination_folder, source_files_in_dir))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files_list,
    install_requires=['setuptools', 'pywebview'],
    zip_safe=True,
    maintainer='iamjunhee',
    maintainer_email='junhee0110@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gemma_server = gemma_ros_client.gemma_service_server_node:main',
            'agricsense_server = gemma_ros_client.agricsense_action_server_node:main',
            'depth_converter_node = gemma_ros_client.depth_image_converter_node:main',
            'annotation_ui_node = gemma_ros_client.annotation_ui_node:main',
            'webview = gemma_ros_client.run_webview:main'
        ],
    },
)