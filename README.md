# Basic Grid Mapping (Log Odds)

Simple mapping for known pose algorithm (ROS package) using log odds. 

Youtube Link: https://www.youtube.com/watch?v=d3tr82qCzv0

[![](https://img.youtube.com/vi/d3tr82qCzv0/0.jpg)](https://www.youtube.com/watch?v=d3tr82qCzv0)

The original map for the recorded bag:

![](/home/salih/catkin_ws/src/basic_grid_mapping/map.png)

## Features

- ROS (outputs `/map` and inputs `/scan`)
- Listens `map`->`base_link` transform.
- Demo bag is available.
- Map is updated when `(x,y)` position changes more than specified threshold.
- Map publish rate is limited.

## Running

```bash
$ roslaunch basic_grid_mapping start.launch
```

