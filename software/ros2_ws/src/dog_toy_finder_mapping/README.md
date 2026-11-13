# Mapping

`dog_toy_finder_mapping` builds a lightweight 2D occupancy map so the robot can
track where it has been.

## Topics

| Topic | Type | Source |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | `occupancy_mapper` |
| `/tracked_path` | `nav_msgs/Path` | `occupancy_mapper` |
| `/detected_toys` | `geometry_msgs/PoseArray` | `occupancy_mapper` |

## Run

```bash
ros2 launch dog_toy_finder_mapping mapping.launch.py
```

Cells the robot travels through are marked free (`0`); unknown cells stay `-1`.
Obstacle cells encode which sensor reported them so the browser can colour them
separately:

| `/map` value | Meaning | Browser colour |
|---|---|---|
| `-1` | unknown | not drawn |
| `0` | free floor | white |
| `60` | toy (occupied space) | green |
| `70` | vision only | blue |
| `85` | both IR and vision | orange |
| `100` | IR only | red |

Vision contributes only while a result is fresh: a `/vision/result` with a
non-`none` `obstacle_ahead` marks a 5x5 cell block at `vision_obstacle_range`
(default 0.8 m) ahead, and expires after `vision_timeout` (default 5 s). IR
marks a 3x3 block at `obstacle_range` (default 0.25 m). When both sensors
report an obstacle at the same time, the near and far regions are both marked
`85` so the fusion is visible. The map is published at 1 Hz; vision can be
disabled with `use_vision:=false`.

`/detected_toys` is a separate `geometry_msgs/PoseArray` layer. When a VLM
result has `target_visible: true` and a `position` of `left`, `center`, or
`right`, the mapper places the toy `distance_m` metres ahead along the robot's
heading, offset `toy_side_offset` metres to that side (default 0.25 m). If the
VLM does not provide a usable `distance_m`, the fallback `toy_forward_distance`
(default 0.6 m) is used; estimates are clamped to `toy_max_distance` (default
5 m). Markers closer than `toy_merge_radius` (default 0.15 m) are merged.

Each toy also marks a `(2 * toy_occupancy_radius + 1)` square of cells as
occupied with value `60`, so the toy appears as occupied space in `/map`
without being confused with IR, vision, or fused obstacles. The browser draws
those cells green and puts a bright green sphere at the estimated centre.
Toy cells are not cleared when the robot drives nearby; they persist as map
memory.

### Parameters

| Parameter | Default | Purpose |
|---|---|---|
| `resolution` | `0.05` | metres per cell |
| `width`, `height` | `200`, `200` | grid size in cells |
| `origin_x`, `origin_y` | `-5.0`, `-5.0` | world position of cell (0, 0) |
| `obstacle_range` | `0.25` | IR obstacle distance ahead (m) |
| `vision_obstacle_range` | `0.8` | VLM obstacle distance ahead (m) |
| `vision_timeout` | `5.0` | seconds a vision result stays fresh (`0` = never expires) |
| `use_vision` | `true` | subscribe to `/vision/result` |
| `ir_topic` | `/ir_obstacle/detected` | IR obstacle Bool topic |
| `publish_period` | `1.0` | seconds between `/map` publishes |
| `toy_forward_distance` | `0.6` | fallback toy distance ahead when the VLM gives none (m) |
| `toy_side_offset` | `0.25` | left/right lateral offset from the heading (m) |
| `toy_max_distance` | `5.0` | clamp for VLM distance estimates (m) |
| `toy_merge_radius` | `0.15` | toys within this distance are treated as the same toy (m) |
| `toy_occupancy_radius` | `1` | cells marked around each toy (1 = 3x3) |
| `toy_max_count` | `500` | maximum stored toy markers |

## View the map

The live map is viewed in a browser. The page, styles, and JavaScript live in
`web/` (`index.html`, `style.css`, `app.js`); `map_webserver.py` only serves
those files and the map data. Vendored Three.js files are in `static/`. The
server listens on all interfaces, so any phone, tablet, or computer on the same
network can open it.

```bash
ros2 run dog_toy_finder_mapping map_webserver
# open http://<computer-ip>:8765
# (override with: ros2 run ... map_webserver --ros-args -p port:=9000)
```

The page updates automatically as `/map`, `/tracked_path`, and `/detected_toys`
change and prints the URL to use in the node log. The server exposes
`/map.json`, `/path.json`, and `/toys.json` for the page. Three.js is bundled
in the package, so the viewing computer does not need internet access. Use
**Save PNG** in the page to download the current view to the computer running
the browser (no file is written on the robot computer). **Reset view** returns
the camera to its default angle.
