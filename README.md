# SimCharts

SimCharts is forked from the SeaCharts package written by Simon Blindheim and maintained by Trym Tengesdal. SeaCharts is a Python-based API for Electronic Navigational Charts (ENC).

SimCharts is developed to be compatible with ROS 2 Humble to get the dvantages of ROS communication. SimCharts is intended to be used as a platform to visualize navigational algorithms for ASVs. SimCharts supply a set of services which can be used to extract staticobstacles from terrain data, represented as polygons. Further, the simulator supports live AIS data from BarentsWatch, which can be used to plot other vessels in the area, but also extract the polygonial shapes to be used for dynamic obstaces.


[![platform](https://img.shields.io/badge/platform-windows-lightgrey)]()
[![platform](https://img.shields.io/badge/platform-linux-lightgrey)]()
[![python version](https://img.shields.io/badge/python-3.9-blue)]()
[![license](https://img.shields.io/badge/license-MIT-green)]()

  

## SeaCharts authors

Simon Blindheim
simon.blindheim@ntnu.no

Trym Tengesdal
trym.tengesdal@ntnu.no



## SimCharts authors

Simon Lexau
simon.lexau@ntnu.no

![](https://github.com/Nagelsaker/simcharts/blob/master/examples/images/Path%20with%20live%20AIS.PNG?raw=true)

Example visualization.  The blue area is the convex set used in a n optimization  A ghost ship is drawn with its corresponding trajectory as well as shadow ships showing the ship's orientation at different waypoints along the trajectory. The other vessels are drawn using live AIS data.

  

## Features

  
- Read and process spatial depth data from [FileGDB](https://gdal.org/drivers/vector/filegdb.html) files into shapefiles.
- Access and manipulate standard geometric shapes such as points and polygon collections.
- Visualize colorful seacharts features and vessels using multiprocessing.
- Visualize nearby vessels over live AIS messages
- The simulator is accessible with ROS 2 services.
- Extract static and dynamic obstacles as polygons.
- Supports paths with shadow ships.
- Live trajectory plotting, and trajectory playback.

  

## Code style

This module follows the [PEP8](https://www.python.org/dev/peps/pep-0008/) convention for Python code.



## Installation

### Anaconda
If you work with Anaconda you will need to speify the correct Python interpreter with shebang at the beginning of your script.

### Linux
Ubuntu v 22.04

Install ROS 2 Humble
``` shell
sudo apt-get update && apt-get install -y \
	curl \
	gnupg2 \
	lsb-release \
	sudo \
	&& curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
	&& echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
	&& apt-get update && apt-get install -y \
	ros-humble-ros-base \
	python3-argcomplete \
	&& rm -rf /var/lib/apt/lists/*
```


Install necessary ROS dependencies
```Shell
sudo apt-get update && apt-get install -y \
	bash-completion \
	build-essential \
	cmake \
	gdb \
	git \
	pylint \
	python3-argcomplete \
	python3-colcon-common-extensions \
	python3-pip \
	python3-rosdep \
	python3-vcstool \
	vim \
	wget \
	ros-humble-ament-lint \
	ros-humble-launch-testing \
	ros-humble-launch-testing-ament-cmake \
	ros-humble-launch-testing-ros \
	python3-autopep8 \
	&& rm -rf /var/lib/apt/lists/* \
	&& rosdep init || echo "rosdep already initialized" \
	&& pip install --upgrade pydocstyle
```

Install the full ROS 2 Humble release
```Shell
sudo apt-get update && apt-get install -y \
	ros-humble-desktop \
	&& rm -rf /var/lib/apt/lists/*
```

Install python dependencies
```Shell
sudo apt install -y libgeos++-dev libgeos3.10.2 libgeos-c1v5 libgeos-dev libgeos-doc \
	&& sudo apt-get install -y python3-pil python3-pil.imagetk \
	&& pip install --no-input matplotlib \
	&& pip install --no-input Shapely==1.8.4 \
	&& pip install --no-input cerberus \
	&& pip install --no-input pyproj \
	&& pip install --no-input Fiona \
	&& pip install --no-input pyshp \
	&& pip install --no-input Pillow \
	&& pip install --no-input pykdtree \
	&& pip install --no-input SciPy \
	&& pip install --no-input OWSLib==0.18 \
	&& pip install --no-input cartopy \
	&& sudo apt-get -y install openssh-server \
	&& pip install --no-input pyqt5 \
	&& apt-get install -y python3-gi python3-gi-cairo gir1.2-gtk-3.0
```

Navigate to your ROS 2 workspace directory and clone the following ros packages into the `/src/` folder
```shell
git clone git@github.com:Nagelsaker/simcharts.git
git clone git@github.com:Nagelsaker/simcharts_interfaces.git
```

If you want to plot vessels with live AIS data, you also need to clone SimCharts AIS Forwarder
```shell
git clone git@github.com:Nagelsaker/simcharts_aisforwarder.git
```

From the root of you ROS 2 workspace directory, run the following line to build the simulator

```shell
source /opt/ros/humble/local_setup.bash
colcon build --merge-all
source install/local_setup.bash
```



## Windows

Install ROS 2 Humble from [here](https://docs.ros.org/en/humble/Installation/Alternatives/Windows-Development-Setup.html), follow the instructions given. This tutorial assumes you installed ROS 2 Humble to `C:\\dev\\ros2_humble`

Create a workspace directory anywhere, for example `C:\\Users\\'user_name'\\ros2`
Inside of which, you need to create a folder called `src`

Install Python 3.8, Pip and Git
Install the following Python packages with Pip

```Shell
pip install --no-input matplotlib
	&& pip install --no-input Shapely==1.8.4
	&& pip install --no-input cerberus
	&& pip install --no-input pyproj
	&& pip install --no-input Fiona
	&& pip install --no-input pyshp
	&& pip install --no-input Pillow
	&& pip install --no-input pykdtree
	&& pip install --no-input SciPy
	&& pip install --no-input OWSLib==0.18
	&& pip install --no-input cartopy
	&& pip install --no-input pyqt5
```

Navigate to your ROS 2 workspace directory and clone the following ros packages into the `/src/` folder
```shell
git clone git@github.com:Nagelsaker/simcharts.git
git clone git@github.com:Nagelsaker/simcharts_interfaces.git
```

If you want to plot vessels with live AIS data, you also need to clone SimCharts AIS Forwarder
```shell
git clone git@github.com:Nagelsaker/simcharts_aisforwarder.git
```

From the root of you ROS 2 workspace directory `C:\\Users\\'user_name'\\ros2` run the following line to build the simulator
```shell
call C:\dev\ros2_humble\local_setup.bat
&& 
cd "C:\\Users\\sjlexau\\ros2_ws_win" 
&&
colcon build --merge install
&& 
call install/local_setup.bat
```

You might also have to run
```Shell
call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat" x86_amd64  
```

and
```Shell
set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

if you run into errors in this step.



# Running SimCharts

This module supports reading and processing `FGDB` files for sea depth data such as the Norwegian coastal data set used for demonstration purposes, found [here](https://kartkatalog.geonorge.no/metadata/2751aacf-5472-4850-a208-3532a51c529a)


### Downloading regional datasets
Follow the above link to download the `Depth data` (`Sjøkart - Dybdedata`) dataset from the [Norwegian Mapping Authority](https://kartkatalog.geonorge.no/?organization=Norwegian%20Mapping%20Authority), by adding it to the Download queue and navigating to the separate [download page](https://kartkatalog.geonorge.no/nedlasting). Choose one or more county areas (e.g. `Trøndelag`), and select the `EUREF89 UTM sone 33, 2d` (`UTM zone 33N`) projection and `FGDB 10.0` format. Finally, select your appropriate user group and purpose, and click `Download` to obtain the ZIP file(s).

Unpack the downloaded file(s) and place the extracted `.gdb` in the `simcharts/data/external/` directory, where the top-level folder `data` is located in the same directory as the launch files.

The `config.yaml` file specifies what ENC data to load and how it will be processed and displayed. The corresponding `config_schema.yaml` specifies the required parameters that must be provided for the software to function properly.

SimCharts can then be started by running:
```shell
ros2 run simcharts simcharts
```

To plot live AIS data run:
```Shell
ros2 run simcharts local_traffic_node
```

Alternatively you can also run the nodes in debug mode:
```Shell
ros2 run simcharts simcharts --ros-args --log-level simcharts__node:=DEBUG

ros2 run simcharts local_traffic_node --ros-args --log-level simcharts__local_traffic_node:=DEBUG
```


# Usage

SimCharts uses ROS 2 for communicating through topics and services. To plot your vessels, and their corresponding paths/trajectories you need to implement clients for the respective services listed below.

| Service                          | Input                                                    | Output                                                                    | Comment                                                                               |
| -------------------------------- | -------------------------------------------------------- | ------------------------------------------------------------------------- | ------------------------------------------------------------------------------------- |
| simcharts__get_dynamic_obstacles | -                                                        | (string) timestamp <br /> (Polygon) dynamic_obstacles                     | Retrieves the coordinates of other vessels's boundaries                               |
| simcharts__get_static_obstacles  | -                                                        | (string) timestamp <br /> (Polygon) static_obstacles                      | Retrieves coordinates of terrain polygons                                             |
| simcharts__get_user_drawn_set    | -                                                        | (float64) timestamp <br /> (Polygon) exterior <br /> (Polygon[]) interior | Retrieves coordinates of the user drawn polygon                                       |
| simcharts__draw_path             | (int64) id <br /> (int64) nrofshadows <br /> (Path) path | -                                                                         | Draws the path for the specified vessel, with nrofshadows shadows vessels             |
| simcharts__draw_trajectory       | (int64) id <br /> (Trajectory) trajectory                | -                                                                         | Draws the trajectory of specified vessel                                              |
| simcharts__draw_obstacle_overlay | (Polygon[]) obstacle_overlay                             | -                                                                         | Draws a custom obstacle overlay to mark obstacles not given in the map                |
| simcharts__add_vessel            | (Vessel) vessel                                          | (bool) was_added                                                          | Adds a vessel to the simulator                                                        |
| simcharts__remove_vessel         | (int64) id                                               | (Vessel) vessel <br /> (bool) was_removed                                 | Removes specified vessel from the simulator                                           |
| simcharts__clean_plot            | -                                                        | -                                                                         | Removes paths, trajectories, obstacle overlays and user drawn sets from the simulator |


The custom datatypes are defined as messages, and presented in the following table

| Message       | Parameters                                                                                                                                                                                                                                                                                                         | Comment |
| ------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------- |
| AIS           | (int64) mmsi <br /> (string) timestamp <br /> (float64) longitude <br /> (float64) latitude <br /> (string) sog <br /> (string) cog <br /> (string) heading <br /> (string) rot <br /> (string) name <br /> (string) shiptype                                                                                      |         |
| ListOfAIS     | (float64) timestamp <br /> (AIS[]) ais_msgs                                                                                                                                                                                                                                                                        |         |
| ListOfVessels | (string) timestamp <br /> (Vessel[]) local_traffic                                                                                                                                                                                                                                                                 |         |
| Path          | (float64[]) x <br /> (float64[]) y <br /> (float64) psi                                                                                                                                                                                                                                                            |         |
| Point         | (float64) x <br /> (float64) y                                                                                                                                                                                                                                                                                     |         |
| Polygon       | (Point[]) points                                                                                                                                                                                                                                                                                                   |         |
| Trajectory    | (float64[]) x <br /> (float64[]) y <br /> (float64[]) psi <br /> (float64[]) t                                                                                                                                                                                                                                     |         |
| Vessel        | (int64) id <br /> (string) timestamp <br /> (float64) x <br /> (float64) y <br /> (float64) sog <br /> (float64) sog <br /> (float64) cog <br /> (float64) heading <br /> (float64) length <br /> (float64) scale <br /> (float64) rot <br /> (string) name <br /> (string) shiptype <br /> (string) vesselsimtype |         |
|               |                                                                                                                                                                                                                                                                                                                    |         |


This simple example shows how to draw a path

```Python
import rclpy
from rclpy.node import Node
from simcharts_interfaces.msg import Path, Vessel
from simcharts_interfaces.srv import DrawPath

class MinimalClient(Node):

    def __init__(self):
        super().__init__('minimal_client')
        draw_path_cli = self.create_client(DrawPath, 'simcharts__draw_path')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = DrawPath.Request()

    def send_request(self, path: Path, id: int = None, nrOfShadows: int = 5) -> None:
		req = DrawPath.Request()
		req.path = path
		req.nrofshadows = nrOfShadows
		req.id = id
		self.future = self.draw_path_cli.call_async(req)
		rclpy.spin_until_future_complete(self, self.future)
		return self.future.result()

def main():
    rclpy.init()
	# Path Object to be added to local traffic
	path = Path()
	path.x = [1,2,3,4,5]
	path.y = [1,2,3,4,5]
	path.psi = [1,2,3,4,5]
	nrOfShadows = 5
	
	# Vessel Object to be added to local traffic
	vessel = Vessel()
	vessel.id = 1001
	vessel.timestamp = "0"
	vessel.x = path.x[0]
	vessel.y = path.y[0]
	vessel.sog = 0.0
	vessel.cog = path.psi[0]
	vessel.heading = path.psi[0]
	vessel.length = 76.2
	vessel.scale = 0.9525
	vessel.rot = 0.0
	vessel.name = "not_implemented_yet"
	vessel.shiptype = "not_implemented_yet"
	vessel.vesselsimtype = "not_implemented_yet"
    
    minimal_client = MinimalClient()
    response = minimal_client.send_request(path, vessel.id, nrOfShadows)
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```



## License

This project uses the [MIT](https://choosealicense.com/licenses/mit/) license.