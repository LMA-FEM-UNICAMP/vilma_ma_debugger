from setuptools import find_packages, setup

package_name = 'vilma_ma_debugger'

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
    maintainer='toffanetto',
    maintainer_email='gabrieltoffanetto@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vilma_ma_status = vilma_ma_debugger.vilma_ma_status:main',
            'vilma_ma_joystick_gui = vilma_ma_debugger.vilma_ma_joystick_gui:main'
        ],
    },
)
