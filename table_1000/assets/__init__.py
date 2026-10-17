"""Shared asset configuration utilities; backend dependencies stay optional."""

from pathlib import Path

ENTRIES = ('object.usda', 'object.usdz', 'robot.usda')


def usd_entry(directory):
    """The USD file to load for a built asset directory: ``object.usda`` (assets with
    behaviors), else ``object.usdz``, else a robot's ``robot.usda``."""
    directory = Path(directory)
    for name in ENTRIES:
        if (directory / name).is_file():
            return directory / name
    raise FileNotFoundError(f'no built USD entry in {directory}')
