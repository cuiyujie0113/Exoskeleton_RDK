from setuptools import setup, find_packages

setup(
    name='sunrise_rdk',
    version='0.1.0',
    packages=find_packages('angle_io','sunrise_rdk'),  
    install_requires=[
        'numpy',
        'opencv-python',
        'h5py',
    ],
    python_requires='>=3.7',
    include_package_data=True,
    entry_points={
        'console_scripts': [
            'data_collector = sunrise_rdk.data_collection.data_collector:main',
            'angle_publisher = sunrise_rdk.angle_io.angle_publisher:main',
            'Exo_angles_pub = sunrise_rdk.angle_io.Exo_angles_pub:main',
            'data_collector_d435i_ros = sunrise_rdk.data_collection.data_collector_D435i_ROS:main',
        ],
    },
    author='Yujie Cui',
    description='Sunrise Robotics Development Kit - I2C angle readout and data collection modules',
    license='MIT',
    classifiers=[
        'Programming Language :: Python :: 3',
        'Operating System :: POSIX :: Linux',
        'License :: OSI Approved :: MIT License',
    ],
)
