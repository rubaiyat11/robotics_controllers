from setuptools import find_packages, setup

package_name = 'cart_pole_lqr_controller'

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
            "cart_pole_lqr_controller_node = cart_pole_lqr_controller.cart_pole_lqr_controller_node:main"
        ],
    },
)
