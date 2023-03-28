from setuptools import setup
from glob import glob
import os

package_name = 'test_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name,'launch'), glob('launch/*')),
        (os.path.join('share', package_name,'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name,'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='uby',
    maintainer_email='uby@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive = test_bot.driving_node:main',
            'pos_vector = test_bot.position:main',
            'go_to_goal = test_bot.go_to_goal:main',
            'video_saver = test_bot.video_saver:main',
            'maze_solver = test_bot.maze_solver:main',
        ],
    },
)
