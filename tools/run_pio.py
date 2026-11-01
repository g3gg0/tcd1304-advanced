"""Use an installed PlatformIO Core or bootstrap a project-local installation."""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import venv


def resolve_pio(project, override):
    if override:
        return override, False

    scripts = "Scripts" if sys.platform == "win32" else "bin"
    executable = "pio.exe" if sys.platform == "win32" else "pio"
    local = project / ".pio" / ("venv-" + sys.platform)
    candidates = (
        (local / scripts / executable, True),
        (Path.home() / ".platformio" / "penv" / scripts / executable, False),
    )
    for candidate, is_local in candidates:
        if candidate.is_file():
            return str(candidate), is_local
    installed = shutil.which("pio") or shutil.which("platformio")
    if installed:
        return installed, False

    print(f"PlatformIO not installed; creating project-local environment: {local}", flush=True)
    venv.EnvBuilder(with_pip=True).create(local)
    python = local / scripts / ("python.exe" if sys.platform == "win32" else "python")
    environment = os.environ.copy()
    environment.setdefault("PIP_CACHE_DIR", str(project / ".pio" / "pip-cache"))
    subprocess.run(
        [str(python), "-m", "pip", "install", "--no-input", "platformio==6.2.0"],
        check=True,
        env=environment,
    )
    return str(local / scripts / executable), True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pio", default="", help="Explicit PlatformIO executable")
    parser.add_argument("args", nargs=argparse.REMAINDER)
    options = parser.parse_args()
    arguments = options.args[1:] if options.args[:1] == ["--"] else options.args
    if not arguments:
        parser.error("a PlatformIO command is required")
    project = Path(__file__).resolve().parent.parent
    pio, is_local = resolve_pio(project, options.pio)
    environment = os.environ.copy()
    if is_local:
        environment.setdefault("PLATFORMIO_CORE_DIR", str(project / ".pio" / ("core-" + sys.platform)))
    return subprocess.call([pio, *arguments], cwd=project, env=environment)


if __name__ == "__main__":
    sys.exit(main())
