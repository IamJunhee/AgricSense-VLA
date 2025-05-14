from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'behave_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ✅ launch 파일 포함
        (os.path.join('share', package_name, 'launch'), glob('behave_package/launch/*.py')),
        # ✅ action 파일 포함
        (os.path.join('share', package_name, 'action'), glob('action/*.action')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_to_goal_node = behave_package.move_to_goal_node:main',
            'annotation_ui_node = behave_package.annotation_ui_node:main',
            #  액션 서버 실행 엔트리포인트
            'move_to_goal_server = behave_package.move_to_goal_server:main',
            'move_to_goal_client = behave_package.move_to_goal_client:main',
        ],
    },
)

