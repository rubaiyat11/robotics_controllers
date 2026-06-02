from setuptools import find_packages, setup

package_name = 'lqr_controller_py'

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
    maintainer='localuser1x',
    maintainer_email='rubaiyatabdullahrabi71@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "lqr_controller_py_node = lqr_controller_py.lqr_controller_py_node:main"
        ],
    },
)
