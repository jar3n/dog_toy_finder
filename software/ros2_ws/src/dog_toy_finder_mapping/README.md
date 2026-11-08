# Mapping

`dog_toy_finder_mapping` builds a lightweight 2D occupancy map so the robot can
track where it has been.

## Topics

| Topic | Type | Source |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | `occupancy_mapper` |
| `/tracked_path` | `nav_msgs/Path` | `occupancy_mapper` |

## Run

```bash
ros2 launch dog_toy_finder_mapping mapping.launch.py
```

Cells the robot travels through are marked free (0); when the IR obstacle
sensor fires, cells a short distance ahead are marked occupied (100); unknown
cells stay -1 (255 in ROS convention? here -1). The map is published at 1 Hz.