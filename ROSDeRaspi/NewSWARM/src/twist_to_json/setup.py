from setuptools import setup

package_name = 'twist_to_json'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raspi',
    maintainer_email='raspi@todo.todo',
    description='Bridge Twist (cmd_vel) to JSON String (/movement)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge = twist_to_json.twist_to_json_node:main',
        ],
    },
)

