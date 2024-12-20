from setuptools import find_packages, setup

with open("aircraftsim/README.md", "r") as f:
    long_description = f.read()

setup(
    name="aircraftsim",
    version="1.0.01",
    description="A wrapper that utilizes JSBSim or Kinematics \
        to simulate aircraft dynamics",
    # package_dir={"": "aircraftsim"},
    # packages=find_packages(where="aircraftsim"),
    packages=find_packages(),
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/jn89b/AircraftSim",
    author="Justin Nguyenvu",
    author_email="jnguyenblue2804@gmail.com",
    license="MIT",
    classifiers=[
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3.10",
        "Operating System :: OS Independent",
    ],
    # install_requires=["bson >= 0.5.10"],
    install_requires=[
        "casadi",
        "JSBSim",
        "matplotlib",
        "numpy",
        "scipy",
        "setuptools",
        "Shapely",
        "simple_pid"],
    extras_require={
        "dev": ["pytest>=7.0", "twine>=4.0.2"],
    },
    python_requires=">=3.10",
)
