"""PlatformIO pre-build hook; generated files stay inside the build directory."""

from pathlib import Path
import os
import sys

Import("env")

project = Path(env.subst("$PROJECT_DIR"))
sys.path.insert(0, str(project / "tools"))
from build_metadata import collect, write_header

board = env.BoardConfig()
toolchain = env.PioPlatform().get_package_dir("toolchain-gccarmnoneeabi")
compiler = (str(Path(toolchain) / "bin" / ("arm-none-eabi-gcc.exe" if os.name == "nt" else "arm-none-eabi-gcc"))
            if toolchain else env.subst("$CC"))
destination = Path(env.subst("$BUILD_DIR")) / "generated"
metadata = collect(
    project,
    version=env.GetProjectOption("custom_firmware_version", "1.1.0"),
    board=env.GetProjectOption("board", "unknown"),
    mcu=board.get("build.mcu", "unknown"),
    environment=env.subst("$PIOENV"),
    cpu_hz=board.get("build.f_cpu", 0),
    compiler=compiler,
)
write_header(destination / "build_info.generated.h", metadata)
env.Append(CPPPATH=[str(destination)], CPPDEFINES=[("HAVE_GENERATED_BUILD_INFO", 1)])

