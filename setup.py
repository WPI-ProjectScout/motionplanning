from setuptools import setup

package_name = 'motionplanning'

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
    maintainer='bs',
    maintainer_email='jpatel@nuhabit.ai',
    description='Local Planner',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = motionplanning.publisher_member_function:main',
            'listener = motionplanning.subscriber_member_function:main',
            'setup_env = motionplanning.setup_env:main',
        ],
    },
    # package_dir={'': 'src'},
)
