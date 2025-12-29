import pathlib


def _get_package_root() -> pathlib.Path:
    """Get the root directory of the seacharts package.

    Returns:
        pathlib.Path: Package root directory (always the actual package, never project root)

    Raises:
        RuntimeError: If package root cannot be determined
    """
    package_file = pathlib.Path(__file__).absolute()
    package_root = package_file.parents[1]

    # If installed in site-packages, we're good
    if "site-packages" in str(package_root) or "dist-packages" in str(package_root):
        return package_root

    # Otherwise, verify it's local development (project root should have config/scenarios)
    project_root = package_root.parent
    if not ((project_root / "pyproject.toml").exists()):
        raise RuntimeError(
            f"Could not determine package root. "
            f"Package file: {package_file}, "
            f"Package root: {package_root}, "
            f"Project root: {project_root}"
        )

    return package_root


package: pathlib.Path = _get_package_root()
is_installed = "site-packages" in str(package) or "dist-packages" in str(package)

if is_installed:
    root = package
else:
    root = pathlib.Path(__file__).parents[2]

config = package / "config.yaml"
config_schema = package / "config_schema.yaml"

data = root / "data"
external = data / "external"
shapefiles = data / "shapefiles"
vessels = data / "vessels.csv"
hazards = data / "hazards"
dynamic = hazards / "dynamic.csv"
static = hazards / "static.csv"
paths = data / "paths"
path1 = paths / "path1.csv"
path2 = paths / "path2.csv"

reports = root / "reports"
frames_dir = reports / "frames"
simulation = reports / "simulation.gif"
frame_files = frames_dir / "frame_*.png"
