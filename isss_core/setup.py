"""colcon/ament_python entry point.

On a bench machine use uv and pyproject.toml instead. Keep name, version and
entry points in step with pyproject.toml; tests/test_packaging.py checks this.
"""

from setuptools import find_packages, setup

PACKAGE = "isss_core"

setup(
    name=PACKAGE,
    version="0.2.0",
    package_dir={"": "src"},
    packages=find_packages("src"),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
    ],
    zip_safe=True,
    description="ROS-free interface for the DeepVision ISSS",
    license="MIT",
    entry_points={
        "console_scripts": [
            "isss-bench = isss.bench:main",
            "isss-report = isss.cli:main_report",
            "isss-stream = isss.cli:main_stream",
            "isss-sim = isss.sim:main",
            "isss-view = isss.cli:main_view",
        ],
    },
)
