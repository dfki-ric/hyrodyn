from setuptools import setup, find_packages

setup(
    name="hyrodyn",
    version="0.1.0",
    description="HyRoDyn robot dynamics library",
    packages=find_packages(),
    package_data={"hyrodyn": ["*.so"]},
    scripts=["scripts/tutorial_hyrodyn.py"],
    install_requires=[
        "numpy",
        "matplotlib",
        "pandas",
        "robomeshcat",
    ],
    python_requires=">=3.10",
)