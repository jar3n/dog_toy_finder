# ROSA agent

ROSA is the high-level LLM brain. It turns a natural-language request into tool
calls that move the robot. The agent code stays separate from the ROS 2 nodes so
the model, prompts, and tools can change without touching the control stack.

## Layout

| File | Purpose |
|---|---|
| `rosa_agent/agent.py` | builds the Ollama model and ROSA instance |
| `rosa_agent/prompts.py` | robot persona, capabilities, and safety instructions |
| `rosa_agent/robot.py` | owns the rclpy node and spins it on a background thread |
| `rosa_agent/tools.py` | agent-facing tools: pose, drive, turn, go-to, stop |
| `rosa_agent/trial_log.py` | writes planning comparison rows to the shared trial CSV |
| `rosa_agent/__main__.py` | interactive command-line loop |
| `tests/test_transforms.py` | pure-Python transform tests |
| `.env.example` | model, endpoint, and approval settings |

The motion implementation itself lives in
`software/ros2_ws/src/dog_toy_finder_motion/`; the tools only validate arguments
and wrap those moves for the LLM.

## Prerequisites

1. Build and source the ROS 2 workspace:

   ```bash
   cd /workspace/software/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

2. Start the micro-ROS agent (on the host, as its own container) and the robot
   bringup:

   ```bash
   docker compose -f docker/docker-compose.yml up -d micro_ros_agent
   ros2 launch dog_toy_finder_bringup robot.launch.py
   ```

3. Confirm `/odometry/filtered` is arriving:

   ```bash
   ros2 topic hz /odometry/filtered
   ```

4. Start Ollama on the computer and make sure the configured model is pulled:

   ```bash
   ollama pull llama3.2
   ollama serve
   ```

## Run

```bash
cd /workspace/software/rosa_agent
set -a; source .env; set +a      # optional, copy .env.example first
python3 -m rosa_agent
```

Example requests:

```text
Where are you?
Drive forward 1 meter.
Turn left 90 degrees.
Go to x 1.0, y 0.5.
Stop.
```

## Comparing LLMs

Run the same request through several models and compare replies and timings:

```bash
python3 -m rosa_agent --compare llama3.2,mistral,qwen2.5
python3 -m rosa_agent --compare llama3.2,mistral --prompt "Drive forward 1 meter."
```

Each comparison model is loaded and unloaded in turn, so only one planner model
uses the Ollama host's VRAM at a time.

Add `--output-csv` to append one row per prompt and model to the shared
planning trial schema (reply, latency, tool calls, and blank ground truth).
Planning trials are written under the repository's `data/planning/<run_id>/`
directory (`/workspace/data/planning` inside the ROS 2 container):

```bash
python3 -m rosa_agent --compare llama3.2,mistral \
    --prompt "Drive forward 1 meter." \
    --output-csv /workspace/data/planning/plan1/planning_trials.csv \
    --run-id plan1
```

Score it later after filling the `gt_*` columns, or pass `--gt-json` now:

```bash
ros2 run dog_toy_finder_data llm_trials score \
    /workspace/data/planning/plan1/planning_trials.csv
```

The ground-truth file may be a flat object that applies to every model:

```json
{"expected_action": "drive_distance", "expected_args": "forward,1",
 "success": true, "notes": "checked on the bench"}
```

or structured by model and prompt:

```json
{
  "default": {"expected_action": "drive_distance", "expected_args": "forward,1"},
  "models": {"mistral": {"expected_args": "1"}},
  "prompts": [
    {"prompt": "Turn left 90 degrees.",
     "ground_truth": {"expected_action": "turn", "expected_args": "left,90"}}
  ]
}
```

`expected_action` and `expected_args` are matched against the reply and the
captured tool calls. `success` overrides those text checks with the operator's
verdict. In the interactive session the same logging applies to every prompt
when `--output-csv` is set; there is no ground truth unless `--gt-json` supplies
it.

Or pick a single model for a session:

```bash
python3 -m rosa_agent --model mistral
```

## Tools

| Tool | Arguments | Behavior |
|---|---|---|
| `get_pose` | none | reports fused x, y, heading |
| `drive_distance` | `direction`, `meters` | forward/backward up to 2 m |
| `turn` | `direction`, `degrees` | left/right up to 180 deg |
| `go_to` | `x`, `y` | drive to a point within 3 m |
| `run_search` | `target`, `mode` | autonomous search: `one` stops at the first toy, `all` finds every toy |
| `check_failsafe` | none | whether the Pico IR failsafe sees an obstacle now |
| `stop_robot` | none | publishes zero `/cmd_vel_auto` (mux forwards it) |
| `describe_scene` | none | describe what the stereo camera sees |
| `find_toy` | `target` | whether a toy is visible, bearing, distance |
| `is_path_clear` | none | whether the path ahead is blocked |

Every move uses fresh `/odometry/filtered` (EKF) feedback and reports what the
fused estimate measured; a missing or stale estimate blocks the move before
anything reaches the motors. If the Pico IR failsafe interrupts a move, the tool
result says so. The robot stops if a tool fails, if the user interrupts, or
when the Pico's 500 ms `/cmd_vel` watchdog expires. The Pico mirrors both
failsafes (IR reflex engage/clear and watchdog stop) to `/pico/log`, and the
motion layer subscribes to `/ir_obstacle/detected` so ROS acts on them.

ROSA's commands go to `/cmd_vel_auto`; the bringup `cmd_vel_mux` forwards them
to the Pico unless keyboard manual drive currently has priority. While manual
control is active, ROSA moves do not reach the motors, and a short release
window follows before autonomous commands resume.

## Environment

| Variable | Default | Meaning |
|---|---|---|
| `OLLAMA_MODEL` | `llama3.2` | model name passed to Ollama |
| `OLLAMA_URL` | `http://localhost:11434` | Ollama endpoint; localhost works with host networking |
| `ROSA_REQUIRE_APPROVAL` | `1` | ask before every move; `0` disables the prompt |

## Tests

```bash
cd /workspace/software/rosa_agent
python3 -m unittest discover -s tests
```

## Safety notes

- Approval is on by default because a hallucinated motion could drive the robot
  into something. Keep it on while developing.
- The prompts forbid claiming motion or sensor readings that no tool reported.
- Vision and autonomous toy search are separate ROS 2 packages and agent tools.
  Add new capabilities as new packages and tools rather than growing `tools.py`
  without bound. The arm is not implemented.
