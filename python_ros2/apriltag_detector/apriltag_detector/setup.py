from setuptools import setup

package_name = 'apriltag_detector'

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
    description='Deteccao AprilTag por 2 cameras, leitura crua por camera',
    license='MIT',
    entry_points={
        'console_scripts': [
            'apriltag_detector_node = apriltag_detector.apriltag_detector_node:main',
        ],
    },
)
