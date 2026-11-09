# End-to-end search workflow tests

These run the real search nodes (`autonomous_search`, `toy_search`)
and the ROSA `run_search` tool against a **fake robot** and **scripted fake
vision**, so no hardware, cameras, or VLMs are needed. The fake robot
integrates `/cmd_vel` into `/odometry/filtered`; the fake vision answers
`/vision/ask` from a script.

## Run everything

From the repo root (builds are cached after the first run):

```bash
./scripts/run_e2e_tests.sh
```

Or one scenario at a time:

```bash
./scripts/run_e2e_tests.sh note_found size
```

The full suite takes several minutes; `rosa_all` / `rosa_multi` drive the
full 3-loop search and take ~2 minutes each.

Manual equivalent (one scenario):

```bash
docker run --rm --network host -v "$PWD":/workspace \
  -w /workspace/software/ros2_ws docker-ros2:latest bash -lc '
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  SCENARIO=distance timeout 300 python3 -u /workspace/tests_e2e/search_workflows.py'
```

## Scenarios

| Scenario | What it proves |
|---|---|
| `note_found` | Fast unit checks: single-target latching, find-all counting, position dedupe, invisible case (both nodes) |
| `size` | Search homes on a `small` toy and stops when it looks `medium` |
| `distance` | Search stops on VLM `distance_m` within `found_distance` |
| `notfound` | Toy never appears: full 4-side coverage, `found=False` |
| `findall` | Two distinct toys found; keeps sweeping (no early exit); exactly two CSV found rows |
| `multitarget` | `target="red ball,blue bone"`: only the visible kind is found, labelled correctly |
| `rosa_one` | ROSA `run_search(target, "one")` stops at the first toy |
| `rosa_all` | ROSA `run_search(target, "all")` finds both scripted toys |
| `rosa_multi` | ROSA `run_search("red ball,blue bone", "all")`: both kinds asked, visible one labelled |

Each scenario prints `PASS`/`FAIL` per check and exits non-zero on failure.
CSV artifacts land in the container's `/tmp/search_e2e_<scenario>.csv` and
`/tmp/rosa_search_<timestamp>.csv` (gone with the container).

How to add a scenario: copy an entry in `search_workflows.py`'s `SCENARIOS`
(with `FakeVision` answers or a `per_target` script), or extend
`rosa_search.py`'s `MODE` handling.
