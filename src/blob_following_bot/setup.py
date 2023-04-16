from setuptools import setup

package_name = 'blob_following_bot'

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
    maintainer='archi',
    maintainer_email='archit.r.jain@gmail.com',
    description='Rasberry Pi Based Moving Object Follower',
    license='MIT Open License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': 
        [
            'blob_detector = blob_following_bot.blob_detector:main',
            'moving_blob_detector = blob_following_bot.moving_blob_detector:main',
            'image_publisher = blob_following_bot.image_publisher:main',
            'follow_blob = blob_following_bot.follow_blob:main',
        ],
    },
)
