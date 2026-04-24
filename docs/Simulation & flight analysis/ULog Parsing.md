---
parent: Simulation & Flight Analysis
title: ULog parsing
---

# Getting ULogs off the drone
The drone records ULogs according to the parameter
[SDLOG_MODE](https://docs.px4.io/main/en/advanced_config/parameter_reference#SDLOG_MODE), and they
exist on the Pixhawk's SD card. To interface with the Pixhawk, we've used `mavproxy.py`, which is
installed on the Jetson (but not in the devcontainer!):
```bash
# Starts the mavproxy console
# Ignore the garbage printed to the screen
mavproxy.py --master=/dev/ttyTHS1 --baudrate 921600

# List all logs
> log list

# Download a specific log
> log download <number>
```

This moves the log from the Pixhawk to the Jetson, but it maxes out at about 70 KiB/s because of the
link between the Jetson and Pixhawk not being meant for fast data transfers. Alternatively you could
take out the sd card and read from it directly.

# ULog parsing
PX4 uses ULog file format to log uORB topics. Parsing is needed to get the logs in readable format.

There are several choices for parsing. Here is presented ones that seem most relevant for our project.

## logs.px4.io
You can upload ULog files to https://logs.px4.io, where it will parse them and plot relevant data.
This is the simplest way to analyze ULog files.

## pyulog

 [Pyulog](https://github.com/PX4/pyulog#scripts) contains python package for parsing ULog files and command line scripts to convert and display them.

#### Install

Install with:
```
pip install pyulog
```

#### Command Line Scripts

All scripts are installed as system-wide applications. They be called on the command line without specifying system path.

Here is listed some relevant scripts.

**Display information from an ULog file (ulog_info):**
```
usage: ulog_info [-h] [-v] file.ulg

positional arguments:
  file.ulg       ULog input file

optional arguments:
  -h, --help     show this help message and exit
  -v, --verbose  Verbose output
```

**Display logged messages from an ULog file (ulog_messages):**
```
usage: ulog_messages [-h] file.ulg

positional arguments:
  file.ulg    ULog input file

optional arguments:
  -h, --help  show this help message and exit
```

**Convert ULog to CSV files (ulog2csv):**
**Note:** Default delimiter is ';' for many spreadsheet apps
```
usage: ulog2csv [-h] [-m MESSAGES] [-d DELIMITER] [-o DIR] file.ulg

positional arguments:
  file.ulg              ULog input file

optional arguments:
  -h, --help            show this help message and exit
  -m MESSAGES, --messages MESSAGES
                        Only consider given messages. Must be a comma-
                        separated list of names, like
                        'sensor_combined,vehicle_gps_position'
  -d DELIMITER, --delimiter DELIMITER
                        Use delimiter in CSV (default is ',')
  -o DIR, --output DIR  Output directory (default is same as input file)
```

**Convert ULog to rosbag files (ulog2rosbag):**
```
usage: ulog2rosbag [-h] [-m MESSAGES] file.ulg result.bag

positional arguments:
  file.ulg              ULog input file
  result.ulg            rosbag output file

optional arguments:
  -h, --help            show this help message and exit
  -m MESSAGES, --messages MESSAGES
                        Only consider given messages. Must be a comma-
                        separated list of names, like
                        'sensor_combined,vehicle_gps_position'
```


## PlotJuggler
[PlotJuggler](https://github.com/facontidavide/PlotJuggler) is more heavyweight tool. It has GUI and tools for visualizing ULog data on a graph.

#### Install

To install in Ubuntu 22.04, run:
```
sudo snap install plotjuggler
```

Windows binary installer: [PlotJuggler-Windows-3.9.3-installer](https://github.com/facontidavide/PlotJuggler/releases/download/3.9.3/PlotJuggler-Windows-3.9.3-installer.exe)

#### Visualing ULog data

Here are steps for visualizing your ULog data.

First load ULog file to PlotJuggler. Then choose and drag data from Timeseries List to graph tab to plot the data. 

You can have multiple data inputs on a same graph or you can have data divided on multiple graphs. 

#### Layout file

You can save your graph layouts on a layout file. Layout file can be used to display the data in same way for every ULog file.

When saving layout to be used for other ULog files be sure to uncheck "Save data source" in the "Save as" window.


## DataComets
[DataComets](https://github.com/dsaffo/DataComets?tab=readme-ov-file) gives good overview of the log data. Simple graphs for all data topics are shown which allows quick analysis and comparison. Flight path is also drawn on a map.

#### Install and Run
Clone the [repo](https://github.com/dsaffo/DataComets?tab=readme-ov-file) and run following commands in the directory.
```
pip install -r requirements.txt
python DataCometsLocal.py
```

DataComets will then run on your local host and can be accessed by going to <http://localhost:5000/> with your browser.

**Note:** if error occurs regarding numpy and pandas import, upgrade the packages.
```
pip install --upgrade numpy pandas
```
