from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'control_planner'


# 1) 收集资源文件
map_files   = [os.path.join('maps',  f) for f in os.listdir('maps')]
param_files = [os.path.join('param', f) for f in os.listdir('param')]
urdf_files  = [os.path.join('urdf',  f) for f in os.listdir('urdf')]



# 2) 定义 data_files
data_files_list = [
    # 注册到 ament resource index
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    # 安装 package.xml 到 share/my_robot
    ('share/' + package_name, ['package.xml']),

    # 安装 maps 目录下的所有文件到 share/my_robot/maps
    ('share/{0}/maps'.format(package_name),   map_files),
    # 安装 param 目录下的所有文件到 share/my_robot/param
    ('share/{0}/param'.format(package_name),  param_files),
    # 安装 urdf 目录下的所有文件到 share/my_robot/urdf
    ('share/{0}/urdf'.format(package_name),   urdf_files),
    # **安装 launch 目录下的所有文件**
    ('share/' + package_name + '/launch', glob('launch/*')),
]


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    # ],
    data_files = data_files_list,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='usv-6081',
    maintainer_email='usv-6081@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_node = control_planner.hello_node:main',
            'usv_Simulation = control_planner.usvSim:main',
            'usv_Simulation_rudder = control_planner.usvSim:main',
            'keyboard_interaction = control_planner.keyboard_interaction:main',
            'publish_joint_angle = control_planner.publish_joint_angle:main',
            'image_subscriber = control_planner.imageSubscriber:main',
        ],
    },
)