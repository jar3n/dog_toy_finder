# Dog Toy Finder Robot

A ROS 2 robot that finds dog toys: a robot arm mounted on a differential-drive platform.

- **Local brain:** Raspberry Pi Pico (micro-ROS)
- **Link:** WiFi (UDP) to a computer running the micro-ROS agent
- **High-level brain:** ROSA (LLM agent) + ROS 2 nodes on the computer
- **Simulation:** Gazebo

## Layout

| Folder | What lives there |
|---|---|
| `firmware/pico_microros/` | Everything that runs on the Pico: drivers, micro-ROS app, WiFi transport, build config |
| `ros2_ws/src/` | Computer-side ROS 2 packages (description, bringup, bridge, navigation, arm, perception, behaviors, messages) |
| `rosa_agent/` | ROSA agent, its tools, prompts, MCP servers, and memory |
| `simulation/` | Gazebo worlds, models, launch files, and sim-specific config |
| `hardware/` | CAD, electronics, wiring/pinouts, bill of materials |
| `docker/` | Containers: `ros2`, `gazebo`, `microros_builder` (plus compose file) |
| `config/` | Shared, non-secret configuration (network, env templates) |
| `scripts/` | Setup, build, flash, and utility helper scripts |
| `data/` | Test images and logs (logs are not tracked) |
| `docs/` | Architecture, setup guides, design notes, diagrams |

## Data flow

```
ROSA agent -> ROS 2 nodes (computer) -> micro-ROS agent --WiFi/UDP--> Pico firmware -> motors / arm / sensors
                    ^
                    +-- Gazebo simulation (stands in for the Pico + hardware)
```

The simulation should expose the same topics as the real robot, so the
computer-side code and ROSA work unchanged against either.
