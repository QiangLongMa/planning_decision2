from setuptools import setup

package_name = 'draw_messages'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'matplotlib'],
    zip_safe=True,
    maintainer='mm',
    maintainer_email='mm@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'real_time_plot = draw_messages.mymessages:main',
        ],
    },
)
