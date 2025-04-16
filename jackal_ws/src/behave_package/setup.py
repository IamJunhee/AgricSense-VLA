from setuptools import find_packages, setup

package_name = 'behave_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]), 
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['behave_package/launch/multi_behavior_launch.py']) #launch 파일을 위한 추가 
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
            'spin_command_node = behave_package.spin_command_node:main',
            'send_spin_command_node = behave_package.send_spin_command_node:main',
            'move_to_goal_node = behave_package.move_to_goal_node:main',
            'move_command_sender = behave_package.move_command_sender:main',
            'move_and_spin_node =behave_package.move_and_spin_node:main',
        ],
    },
)
