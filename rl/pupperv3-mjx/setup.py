from setuptools import setup, find_packages

# Read requirements from requirements.txt
with open("requirements.txt") as f:
    requirements = f.read().splitlines()

setup(
    name="pupperv3_mjx",  # Replace with your own package name
    version="0.1",  # Package version
    packages=find_packages(),  # Automatically find package modules
    install_requires=requirements,  # Use the requirements from requirements.txt
    # Optional fields:
    author="Nathan Kau, Google",
    author_email="nathan.kau@gmail.com",
    description="A short description of your project",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",  # if your README is Markdown
    python_requires=">=3.10",  # Minimum version requirement of the package
)
