from setuptools import setup

package_name = 'pingpong_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liqsuq',
    maintainer_email='liqsuq@rkgk.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'pingpong_gui = pingpong_gui.pingpong_gui:main',
    ],},
)
