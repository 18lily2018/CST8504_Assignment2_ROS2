from setuptools import setup

package_name = 'aisd_create3_lab5'

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
    maintainer='aisd',
    maintainer_email='your_email@example.com',
    description='Lab 5 – Create3 bumper to LED demo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bumper_led = aisd_create3_lab5.bumper_led:main',
        ],
    },
)
