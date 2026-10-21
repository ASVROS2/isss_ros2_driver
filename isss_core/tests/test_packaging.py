"""pyproject.toml (uv) and setup.py (colcon) describe the same package; keep them in step."""

import ast
import re
from pathlib import Path

import isss

ROOT = Path(__file__).parent.parent


def _setup_kwargs() -> dict:
    tree = ast.parse((ROOT / "setup.py").read_text())
    call = next(n for n in ast.walk(tree) if isinstance(n, ast.Call) and getattr(n.func, "id", "") == "setup")
    return {kw.arg: kw.value for kw in call.keywords}


def _pyproject_value(key: str) -> str:
    return re.search(rf'^{key} = "([^"]+)"', (ROOT / "pyproject.toml").read_text(), re.M).group(1)


def test_versions_match():
    version = ast.literal_eval(_setup_kwargs()["version"])
    package_xml = re.search(r"<version>([^<]+)</version>", (ROOT / "package.xml").read_text()).group(1)
    assert version == _pyproject_value("version") == package_xml == isss.__version__


def test_console_scripts_match():
    text = (ROOT / "pyproject.toml").read_text()
    section = text.split("[project.scripts]")[1].split("\n[")[0]
    pyproject_scripts = sorted(
        f"{name} = {target}" for name, target in re.findall(r'^([\w-]+) = "([^"]+)"', section, re.M)
    )
    setup_scripts = sorted(ast.literal_eval(_setup_kwargs()["entry_points"])["console_scripts"])
    assert pyproject_scripts == setup_scripts
