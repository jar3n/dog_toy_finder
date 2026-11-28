"""Persist ROSA planning trials for LLM comparison.

The vision side is recorded by ``vision_eval`` when it replays saved captures.
This module does the same job for the planner: it turns one ROSA reply into one
row of the shared ``planning_trials.csv`` schema from
``dog_toy_finder_data.llm_trials``.

Ground truth is optional at collection time. Without it the row still records
the model, prompt, reply, latency, and the tools the model called; fill the
``gt_*`` columns later in a spreadsheet, or supply a JSON file:

    {
      "default": {"expected_action": "drive_distance", "expected_args": "forward,1"},
      "models": {
        "mistral": {"expected_action": "drive_distance", "expected_args": "1"}
      }
    }

A flat object such as ``{"expected_action": "...", "success": true}`` applies
to every model. The file may also hold a ``prompts`` list whose entries match
a prompt and carry their own ground truth, which is useful when one comparison
run uses several prompts.
"""

import json
import os

from dog_toy_finder_data.llm_trials import (
    PLANNING_FIELDS,
    append_csv,
    planning_row,
)


def load_ground_truth(path):
    """Load a ground-truth JSON file, returning ``{}`` when no path is given."""
    if not path:
        return {}
    with open(path) as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError("ground-truth JSON must be an object: %s" % path)
    return data


def ground_truth_for(data, model, prompt=""):
    """Return the ground-truth dict that applies to one model and prompt.

    ``data`` may be flat, or have ``default``/``models``/``prompts`` sections.
    Later matches override earlier ones, so a prompt entry can refine a model
    default.
    """
    if not isinstance(data, dict) or not data:
        return {}
    structured = any(key in data for key in ("default", "models", "prompts"))
    merged = {}
    if structured:
        if isinstance(data.get("default"), dict):
            merged.update(data["default"])
        model_entries = data.get("models")
        if isinstance(model_entries, dict) and isinstance(
                model_entries.get(model), dict):
            merged.update(model_entries[model])
        prompt_entries = data.get("prompts")
        if isinstance(prompt_entries, list):
            for entry in prompt_entries:
                if not isinstance(entry, dict) or entry.get("prompt") != prompt:
                    continue
                if isinstance(entry.get("ground_truth"), dict):
                    merged.update(entry["ground_truth"])
                elif isinstance(entry.get("models"), dict) and isinstance(
                        entry["models"].get(model), dict):
                    merged.update(entry["models"][model])
    else:
        merged.update(data)
    return merged


def record_planning_trial(path, model, prompt, reply, elapsed_s, run_id,
                          tool_calls=None, error="", ground_truth=None):
    """Append one planning trial row and return it.

    ``tool_calls`` is the list captured from ROSA's streaming events. The CSV
    path is created if needed and receives its header on the first row.
    """
    if not path:
        return None
    row = planning_row(model, prompt, reply, elapsed_s, run_id,
                       tool_calls=tool_calls, error=error,
                       ground_truth=ground_truth)
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    append_csv(path, row, PLANNING_FIELDS)
    return row
