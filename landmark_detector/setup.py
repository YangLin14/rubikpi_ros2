from glob import glob

from setuptools import find_packages, setup

package_name = 'landmark_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/models', glob('models/*.onnx') + glob('models/*.txt')),
    ],
    install_requires=['setuptools', 'numpy', 'onnxruntime', 'pyyaml'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='chl235@ucsd.edu',
    description='YOLO object detector for visual landmarks',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detector_node = landmark_detector.detector_node:main',
            'benchmark = landmark_detector.benchmark:main',
            'dataset_capture = landmark_detector.dataset_capture:main',
            'range_node = landmark_detector.range_node:main',
        ],
    },
)
