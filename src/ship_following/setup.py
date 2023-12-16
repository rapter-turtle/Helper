from setuptools import setup

package_name = 'ck_joy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ck',
    maintainer_email='ck@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'ck_joy_f = ck_joy.usv_turn_final_first:main',
        'ck_joy_s = ck_joy.usv_turn_final_second:main',
        ],
    },
)
