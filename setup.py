from setuptools import find_packages, setup

package_name = 'priority_demos_py'

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
    maintainer='yundo',
    maintainer_email='rikochet@kakao.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_priority = priority_demos_py.node_priority:main',
            'timer_priority = priority_demos_py.timer_priority:main',
            'subscription_priority = priority_demos_py.subscription_priority:main',
        ],
    },
)
