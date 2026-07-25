from setuptools import setup, find_packages

setup(
    name='nav_core',
    version='1.0.0',
    description='Platform-agnostic navigation kernel',
    packages=find_packages(),
    py_modules=[],  # re-exports from parent dir
    python_requires='>=3.8',
)
