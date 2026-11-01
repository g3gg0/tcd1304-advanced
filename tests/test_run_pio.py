from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import run_pio


class RunPioTests(unittest.TestCase):
    def test_override_does_not_install(self):
        with patch.object(run_pio.venv, "EnvBuilder") as builder:
            self.assertEqual(run_pio.resolve_pio(Path("project"), "custom pio"), ("custom pio", False))
            builder.assert_not_called()

    def test_local_installation_is_preferred(self):
        with patch.object(run_pio.sys, "platform", "linux"), \
                patch.object(Path, "is_file", return_value=True):
            self.assertEqual(
                run_pio.resolve_pio(Path("project"), ""),
                (str(Path("project/.pio/venv-linux/bin/pio")), True),
            )

    def test_user_installation_is_used(self):
        with patch.object(run_pio.sys, "platform", "win32"), \
                patch.object(Path, "home", return_value=Path("user")), \
                patch.object(Path, "is_file", side_effect=[False, True]):
            self.assertEqual(
                run_pio.resolve_pio(Path("project"), ""),
                (str(Path("user/.platformio/penv/Scripts/pio.exe")), False),
            )

    def test_path_installation_is_used(self):
        with patch.object(Path, "is_file", return_value=False), \
                patch.object(run_pio.shutil, "which", side_effect=[None, "platformio"]):
            self.assertEqual(run_pio.resolve_pio(Path("project"), ""), ("platformio", False))

    def test_missing_installation_bootstraps_locally(self):
        for platform, scripts, executable, python in (
            ("linux", "bin", "pio", "python"),
            ("win32", "Scripts", "pio.exe", "python.exe"),
        ):
            with self.subTest(platform=platform), \
                    patch.object(run_pio.sys, "platform", platform), \
                    patch.object(Path, "home", return_value=Path("user")), \
                    patch.object(Path, "is_file", return_value=False), \
                    patch.object(run_pio.shutil, "which", return_value=None), \
                    patch.object(run_pio.venv, "EnvBuilder") as builder, \
                    patch.dict(run_pio.os.environ, {}, clear=True), \
                    patch.object(run_pio.subprocess, "run") as install:
                local = Path("project/.pio") / ("venv-" + platform)
                self.assertEqual(
                    run_pio.resolve_pio(Path("project"), ""),
                    (str(local / scripts / executable), True),
                )
                builder.assert_called_once_with(with_pip=True)
                builder.return_value.create.assert_called_once_with(local)
                install.assert_called_once_with(
                    [str(local / scripts / python), "-m", "pip", "install", "--no-input", "platformio==6.2.0"],
                    check=True,
                    env={"PIP_CACHE_DIR": str(Path("project/.pio/pip-cache"))},
                )

    def test_installation_failure_is_not_hidden(self):
        with patch.object(Path, "is_file", return_value=False), \
                patch.object(run_pio.shutil, "which", return_value=None), \
                patch.object(run_pio.venv, "EnvBuilder"), \
                patch.object(run_pio.subprocess, "run", side_effect=subprocess.CalledProcessError(1, "pip")):
            with self.assertRaises(subprocess.CalledProcessError):
                run_pio.resolve_pio(Path("project"), "")

    def test_local_core_directory_and_exit_status(self):
        with patch.object(sys, "argv", ["run_pio.py", "--", "run", "-e", "nucleo_f411re"]), \
                patch.object(run_pio, "resolve_pio", return_value=("local-pio", True)), \
                patch.dict(run_pio.os.environ, {}, clear=True), \
                patch.object(run_pio.subprocess, "call", return_value=7) as call:
            self.assertEqual(run_pio.main(), 7)
            project = Path(run_pio.__file__).resolve().parent.parent
            call.assert_called_once_with(
                ["local-pio", "run", "-e", "nucleo_f411re"],
                cwd=project,
                env={"PLATFORMIO_CORE_DIR": str(project / ".pio" / ("core-" + sys.platform))},
            )

    def test_global_platformio_option_is_forwarded(self):
        with patch.object(sys, "argv", ["run_pio.py", "--pio", "custom-pio", "--", "--version"]), \
                patch.object(run_pio, "resolve_pio", return_value=("custom-pio", False)), \
                patch.object(run_pio.subprocess, "call", return_value=0) as call:
            self.assertEqual(run_pio.main(), 0)
            self.assertEqual(call.call_args.args[0], ["custom-pio", "--version"])


if __name__ == "__main__":
    unittest.main()
