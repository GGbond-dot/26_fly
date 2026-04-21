import shutil
import subprocess
from dataclasses import dataclass


@dataclass(frozen=True)
class MagnetLevels:
    on: int = 0
    off: int = 1


class GpioMagnet:
    def __init__(self, pin: int = 10, levels: MagnetLevels | None = None) -> None:
        self.pin = pin
        self.levels = levels or MagnetLevels()

    def check_available(self) -> bool:
        return shutil.which("gpio") is not None

    def set_output_mode(self) -> None:
        self._run("mode", str(self.pin), "out")

    def set_enabled(self, enabled: bool) -> None:
        self.set_output_mode()
        level = self.levels.on if enabled else self.levels.off
        self._run("write", str(self.pin), str(level))

    def read_enabled(self) -> bool:
        self.set_output_mode()
        result = self._run("read", str(self.pin), capture=True)
        return result.stdout.strip() == str(self.levels.on)

    def _run(self, *args: str, capture: bool = False) -> subprocess.CompletedProcess[str]:
        if not self.check_available():
            raise RuntimeError("gpio command not found. Please install/enable WiringOP first.")

        return subprocess.run(
            ["gpio", *args],
            check=True,
            text=True,
            capture_output=capture,
        )
