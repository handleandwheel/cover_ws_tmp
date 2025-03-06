from setuptools import setup, find_packages

setup(
    name="mstc",  # Replace with your package name
    version="0.1.0",  # Initial release version
    author="Your Name",
    author_email="your.email@example.com",
    description="A brief description of your package",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/yourusername/mypackage",  # Replace with your repo URL
    packages=find_packages(),  # Automatically find all packages
    install_requires=[],
    # classifiers=[
    #     "Programming Language :: Python :: 3",
    #     "License :: OSI Approved :: MIT License",
    #     "Operating System :: OS Independent",
    # ],
    python_requires=">=3.7",  # Minimum Python version requirement
    # entry_points={
    #     "console_scripts": [
    #         "mypackage-cli=mypackage.cli:main",  # Example command-line script
    #     ],
    # },
)
