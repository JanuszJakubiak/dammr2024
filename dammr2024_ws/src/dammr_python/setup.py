from setuptools import setup

package_name = 'dammr_python'

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
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_world = dammr_python.hello_world:main',
            'str_publisher = dammr_python.str_publisher:main',
            'dwm_publisher = dammr_python.dwm_publisher:main',
        ],
    },
)
