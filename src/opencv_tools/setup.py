from setuptools import setup

package_name = 'opencv_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Alternative OpenCV tools for camera streaming',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webcam_pub = opencv_tools.webcam_pub:main',
        ],
    },
)
