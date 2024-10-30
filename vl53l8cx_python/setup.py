
from setuptools import setup


with open("README.md", "r") as fh:
    long_description = fh.read()


setup(
    name="VL53L8CX",
    version="1.0.1",
    author="Rhinos",
    description="Python native implementation of VL53L8CX ultra light driver code",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/Abstract-Horizon/vl53l5cx_python",
    packages=['vl53l8cx'],

    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)
