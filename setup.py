from setuptools import setup, find_packages

if __name__ == "__main__":
    setup(
        # ds, ff, ie
        name="facade",
        version="0.21.0",
        author="laurelkeys",
        url="https://github.com/laurelkeys/ff",
        packages=find_packages(where="src"),
        package_dir={"": "src"},
        python_requires=">=3.8",
    )
