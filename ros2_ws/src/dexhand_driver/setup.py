from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'dexhand_driver'

setup(
    name=package_name,
    version='0.0.1',
    # 自动查找所有包，包含 dexhand_driver 和它下面的 pyzlg_dexhand
    packages=find_packages(exclude=['test']), 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'python-can'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='DexHand ROS 2 Driver',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 注册节点入口：包名.文件名:函数名
            'hardware_node = dexhand_driver.hardware_node:main',
        ],
    },
)