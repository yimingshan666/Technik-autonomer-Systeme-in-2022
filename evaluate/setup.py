from setuptools import setup

package_name = 'evaluate'

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
    maintainer='user',
    maintainer_email='ge23jil@mytum.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
       		 'eval = evaluate.for_evaluate:main',
             'goal = evaluate.pub_goal:main',
             'goal2 = evaluate.pub_goal2:main',
             'savedata = evaluate.save_data:main',
             'evalall = evaluate.eval_all:main'

        ],
    },
)
