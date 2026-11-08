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
sensor fires, cells a short distance ahead are marked occupied (100); when the
vision node reports a central obstacle, a larger block further ahead is marked
occupied. Unknown cells stay -1. The map is published at 1 Hz. Vision can be
disabled with `use_vision:=false`.

## View the map

- Web viewer (3D, in a browser):
  ```bash
  ros2 run dog_toy_finder_mapping map_webserver
  # open http://<computer-ip>:8765
  # (override with: ros2 run ... map_webserver --ros-args -p port:=9000)
  ```
- Save to PGM+YAML: `ros2 run dog_toy_finder_mapping save_map -o /tmp/my_map`
- ASCII view: `ros2 run dog_toy_finder_mapping view_map`