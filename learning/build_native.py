"""Build the local Python/MuJoCo native extension (Linux OpenMP, macOS serial)."""
from __future__ import annotations
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import sys
import sysconfig
import mujoco
import pybind11


def build(force: bool = False) -> Path:
    root = Path(__file__).resolve().parent
    source = root / "native.cpp"
    output = root / ("_sweetie_native" + sysconfig.get_config_var("EXT_SUFFIX"))
    package = Path(mujoco.__file__).resolve().parent
    libraries = sorted(package.glob("libmujoco.so*")) + sorted(package.glob("libmujoco*.dylib"))
    if not libraries:
        raise RuntimeError(f"Cannot locate the MuJoCo library in {package}")
    if output.exists() and not force and output.stat().st_mtime > max(source.stat().st_mtime, Path(__file__).stat().st_mtime):
        return output
    temporary=output.with_name(output.stem+f".{os.getpid()}.tmp"+output.suffix)
    compiler = shlex.split(os.environ.get("CXX", "c++" if sys.platform == "darwin" else "g++"))
    command = compiler + ["-O3", "-DNDEBUG", "-std=c++17", "-shared", "-fPIC",
        f"-I{pybind11.get_include()}", f"-I{sysconfig.get_path('include')}", f"-I{package / 'include'}",
        str(source), str(libraries[0]), f"-Wl,-rpath,{package}", "-o", str(temporary)]
    if sys.platform == "darwin":
        command += ["-undefined", "dynamic_lookup", "-Wl,-headerpad_max_install_names"]
    else:
        command += ["-fopenmp"]
    print(shlex.join(command), flush=True)
    subprocess.run(command, check=True)
    if sys.platform == "darwin":
        # PyPI's dylib keeps the upstream framework install name although the
        # wheel stores it directly in the package directory.
        identity = subprocess.check_output(["otool", "-D", str(libraries[0])], text=True).splitlines()[1].strip()
        subprocess.run(["install_name_tool", "-change", identity, str(libraries[0]), str(temporary)], check=True)
    os.replace(temporary,output)
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force", action="store_true")
    print(build(parser.parse_args().force))
