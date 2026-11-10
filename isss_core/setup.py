"""colcon/ament_python entry point.

On a bench machine use uv and pyproject.toml instead. Keep name, version and
entry points in step with pyproject.toml; tests/test_packaging.py checks this.
"""

from setuptools import find_packages, setup
from setuptools.dist import Distribution


class _Distribution(Distribution):
    """Keep ``python_requires`` (pyproject's ``requires-python``) a plain string.

    setuptools 68 makes it a ``SpecifierSet``, whose repr colcon-python-setup-py
    0.2.9 cannot read back (``ast.literal_eval``), so colcon failed at discovery.
    """

    def parse_config_files(self, *args, **kwargs):
        super().parse_config_files(*args, **kwargs)
        for owner in (self, self.metadata):
            if getattr(owner, "python_requires", None) is not None:
                owner.python_requires = str(owner.python_requires)


PACKAGE = "isss_core"

setup(
    distclass=_Distribution,
    name=PACKAGE,
    version="0.2.0",
    package_dir={"": "src"},
    packages=find_packages("src"),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
    ],
    zip_safe=True,
    tests_require=["pytest"],  # colcon test runs pytest (not unittest) only when this names it
    description="ROS-free interface for the DeepVision ISSS",
    license="MIT",
    entry_points={
        "console_scripts": [
            "isss-bench = isss.bench:main",
            "isss-export = isss.cli:main_export",
            "isss-export-legacy = isss.cli:main_export_legacy",
            "isss-report = isss.cli:main_report",
            "isss-stream = isss.cli:main_stream",
            "isss-sim = isss.sim:main",
            "isss-view = isss.cli:main_view",
            "isss-view-legacy = isss.cli:main_view_legacy",
        ],
    },
)
