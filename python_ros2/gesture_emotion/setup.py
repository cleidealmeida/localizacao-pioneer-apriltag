import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'gesture_emotion'


def data_files_recursive(base):
    """Instala models/ e Base_de_dados/ junto com o pacote."""
    paths = []
    for root, _, files in os.walk(base):
        if files:
            dest = os.path.join('lib/python3.10/site-packages', root)
            paths.append((dest, [os.path.join(root, f) for f in files]))
    return paths


setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    + data_files_recursive(os.path.join(package_name, 'Emotion_GestureDetector', 'models'))
    + data_files_recursive(os.path.join(package_name, 'Emotion_GestureDetector', 'Base_de_dados')),
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Cleide Almeida Coelho Fernandes',
    maintainer_email='cleide@ufv.br',
    description='Gestos (KNN) + emocoes (CNN) publicando em /Gesture',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gesture_emotion_node = gesture_emotion.gesture_emotion_node:main',
        ],
    },
)
