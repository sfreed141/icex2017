"""
setup for sonar_tools module
"""

from setuptools import setup

setup(
    name='sonar_tools',
    version='0.1',
    description='Collection of tools for processing sonar data from jsf files',
    url='',
    author='Sam Freed',
    author_email='sfreed141@gmail.com',
    license='MIT',
    packages=['sonar_tools'],
    install_requires=[
        'numpy',
    ],
    zip_safe=False
)
