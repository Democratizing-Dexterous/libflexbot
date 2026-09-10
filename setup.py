from pathlib import Path
import shutil
import subprocess

from setuptools import Extension, find_packages, setup
from setuptools.command.build_ext import build_ext


ROOT = Path(__file__).resolve().parent


class CMakeBuild(build_ext):
    def run(self):
        build_dir = ROOT / "build-pip"
        build_dir.mkdir(parents=True, exist_ok=True)

        configuration = "Debug" if self.debug else "Release"
        subprocess.check_call(
            ["cmake", "-S", str(ROOT), "-B", str(build_dir), f"-DCMAKE_BUILD_TYPE={configuration}"]
        )
        subprocess.check_call(
            [
                "cmake",
                "--build",
                str(build_dir),
                "--config",
                configuration,
                "--target",
                "_libflexbot",
                "--clean-first",
                "--parallel",
            ]
        )

        source_package = build_dir / "python" / "libflexbot"
        source_extension = source_package / "_libflexbot.so"
        if not source_extension.exists():
            built_extensions = sorted(source_package.glob("_libflexbot*.so"))
            source_extension = built_extensions[0] if built_extensions else None
        if source_extension is None or not source_extension.exists():
            raise RuntimeError("CMake did not produce python/libflexbot/_libflexbot*.so")

        destination = Path(self.get_ext_fullpath("libflexbot._libflexbot"))
        destination.parent.mkdir(parents=True, exist_ok=True)
        for stale_extension in destination.parent.glob("_libflexbot*.so"):
            if stale_extension.resolve() != destination.resolve():
                stale_extension.unlink()
        if source_extension.resolve() != destination.resolve():
            shutil.copy2(source_extension, destination)
            if source_extension.name == "_libflexbot.so":
                source_extension.unlink()

        bundled_driver = source_package / "libcontrolcanfd.so"
        if not bundled_driver.exists():
            raise RuntimeError("Missing bundled driver in CMake output")
        installed_driver = destination.parent / "libcontrolcanfd.so"
        if bundled_driver.resolve() != installed_driver.resolve():
            shutil.copy2(bundled_driver, installed_driver)


setup(
    name="libflexbot",
    version="0.1.0",
    description="Python SDK for LibFlexBot CAN-FD motor control",
    package_dir={"": "python"},
    packages=find_packages("python"),
    package_data={"libflexbot": ["libcontrolcanfd.so"]},
    include_package_data=False,
    ext_modules=[Extension("libflexbot._libflexbot", sources=[])],
    cmdclass={"build_ext": CMakeBuild},
    entry_points={
        "console_scripts": [
            "flexcli=libflexbot.cli:main",
            "flexzero=libflexbot.cli:zero_main",
        ]
    },
    python_requires=">=3.8",
    install_requires=["numpy"],
)
