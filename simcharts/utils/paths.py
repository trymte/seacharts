import pathlib

# Defines (hard-coded paths to relevant files)
ros2_ws = pathlib.Path(__file__).parents[5]
root = ros2_ws / 'src' / 'simcharts'
package = root / 'simcharts'

config = package / 'config.yaml'
config_schema = package / 'config_schema.yaml'

data = root / 'data'
external = data / 'external'
shapefiles = data / 'shapefiles'

vessels = data / 'vessels.csv'

hazards = data / 'hazards'
dynamic = hazards / 'dynamic.csv'
static = hazards / 'static.csv'

paths = data / 'paths'
path1 = paths / 'path1.csv'
path2 = paths / 'path2.csv'

reports = root / 'reports'
frames_dir = reports / 'frames'
simulation = reports / 'simulation.gif'
frame_files = frames_dir / 'frame_*.png'
