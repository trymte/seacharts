"""Unit tests for enc GDB data file path resolution functionality."""

import tempfile
from pathlib import Path

from seacharts.utils import files


def test_resolve_relative_path():
    relative_path = "test_data.gdb"
    resolved = files.resolve_file_path(relative_path)

    assert resolved.is_absolute()
    assert "data/external" in str(resolved)
    assert resolved.name == relative_path


def test_resolve_absolute_path():
    with tempfile.TemporaryDirectory() as tmpdir:
        absolute_path = Path(tmpdir) / "test_data.gdb"
        absolute_path.mkdir()

        resolved = files.resolve_file_path(str(absolute_path))

        assert resolved == absolute_path
        assert resolved.is_absolute()


def test_resolve_tilde_path():
    tilde_path = "~/enc_data/Rogaland_utm33.gdb"
    resolved = files.resolve_file_path(tilde_path)

    assert resolved.is_absolute()
    assert (
        Path.home() in resolved.parents
        or resolved == Path.home() / "enc_data" / "Rogaland_utm33.gdb"
    )
    assert resolved.name == "Rogaland_utm33.gdb"


def test_resolve_path_from_path_home():
    path_from_home = str(Path.home() / "enc_data" / "Rogaland_utm33.gdb")
    resolved = files.resolve_file_path(path_from_home)

    assert resolved.is_absolute()
    assert resolved == Path(path_from_home)
    assert "enc_data" in str(resolved)
    assert resolved.name == "Rogaland_utm33.gdb"
