"""colcon/ament_python entry point.

On a bench machine use uv and pyproject.toml instead. Keep name, version and
entry points in step with pyproject.toml; tests/test_packaging.py checks this.
"""

from setuptools import find_packages, setup

PACKAGE = "isss_proc"

setup(
    name=PACKAGE,
    version="0.1.0",
    package_dir={"": "src"},
    packages=find_packages("src"),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
    ],
    zip_safe=True,
    description="Processing of DeepVision ISSS pings (ROS-free)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "isss-proc = isss_proc.cli:main_process",
            "isss-proc-view = isss_proc.cli:main_view",
            "isss-proc-crosstalk = isss_proc.cli:main_crosstalk",
        ],
    },
)
