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

The live map is viewed in a browser. The page, styles, and JavaScript live in
`web/` (`index.html`, `style.css`, `app.js`); `map_webserver.py` only serves
those files and the map data. Vendored Three.js files are in `static/`. The server listens on all interfaces, so
any phone, tablet, or computer on the same network can open it.

```bash
ros2 run dog_toy_finder_mapping map_webserver
# open http://<computer-ip>:8765
# (override with: ros2 run ... map_webserver --ros-args -p port:=9000)
```

The page updates automatically as `/map` and `/tracked_path` change and prints
the URL to use in the node log. Three.js is bundled in the package, so the
viewing computer does not need internet access. Use **Save PNG** in the page to
download the current view to the computer running the browser (no file is
written on the robot computer). **Reset view** returns the camera to its
default angle.
