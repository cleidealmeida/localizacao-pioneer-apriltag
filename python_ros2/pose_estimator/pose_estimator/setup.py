from setuptools import setup

package_name = 'pose_estimator'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Cleide Almeida Coelho Fernandes',
    maintainer_email='cleide@ufv.br',
    description='Triangulacao + Kalman -> /robot_pose',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pose_estimator_node = pose_estimator.pose_estimator_node:main',
        ],
    },
)
