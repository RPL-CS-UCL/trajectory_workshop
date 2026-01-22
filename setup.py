from setuptools import setup


setup(
    name="traj_lib",
    version="1.0.0",
    package_dir={"": "src"},
    python_requires=">=3.10, <4",
    install_requires=["matplotlib==3.9.*"],
)
