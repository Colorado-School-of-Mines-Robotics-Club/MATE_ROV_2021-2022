from setuptools import setup
from glob import glob

package_name = 'bs_flask'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/templates/', glob("templates/*")),
        ('share/' + package_name + '/static/', glob("static/*")),
        ('share/' + package_name, [f"{package_name}/default.jpg",f"{package_name}/stylesheet.css"])

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yameat',
    maintainer_email='zac71113@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bs_flask = bs_flask.bs_flask:main'
        ],
    },
)
