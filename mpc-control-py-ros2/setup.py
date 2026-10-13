from setuptools import setup

package_name = 'mpc_control_py_ros2'

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
    maintainer='johannes',
    maintainer_email='johannes.waibel@epfl.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc_control_py_node2 = mpc_control_py_ros2.mpc_control_py_node2:main',
            'spline_following_mpc_MAMH_pynode2 = mpc_control_py_ros2.spline_following_mpc_MAMH_pynode2:main',
            'spline_following_mpc_MRMF_pynode2 = mpc_control_py_ros2.spline_following_mpc_MRMF_pynode2:main',
            'spline_following_tire_mpc_MAMH_pynode2 = mpc_control_py_ros2.spline_following_tire_mpc_MAMH_pynode2:main',
            'spline_following_tire_mpc_MAMH_worker = mpc_control_py_ros2.spline_following_tire_mpc_MAMH_worker:main',
            'spline_following_tire_mpc_MRMF_pynode2 = mpc_control_py_ros2.spline_following_tire_mpc_MRMF_pynode2:main',
        ],
    },
)
