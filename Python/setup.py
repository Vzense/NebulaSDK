import os
from setuptools import setup

version = os.environ.get("PKG_VERSION", "0.1")

setup(
    name="nebula",
    version=version,
    description="Nebula SDK source code",
    author="Vzense Technology",
    author_email="info@vzense.com",
    url='https://github.com/Vzense/NebulaSDK',
    packages=["API"],
    include_package_data=True,
    license="BSD",
    install_requires=['numpy'],
)
