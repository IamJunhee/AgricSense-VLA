from setuptools import find_packages, setup

package_name = 'gemma_ros_client'

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
    maintainer='root', # 작성자 정보 수정
    maintainer_email='junhee0110@gmail.com', # 작성자 정보 수정
    description='TODO: Package description', # 설명 추가
    license='TODO: License declaration', # 라이선스 선택
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gemma_server = gemma_ros_client.gemma_service_server_node:main',
            'agricsense_server = gemma_ros_client.agricsense_action_server_node:main',
            'depth_converter_node = gemma_ros_client.depth_image_converter_node:main',
        ],
    },
)