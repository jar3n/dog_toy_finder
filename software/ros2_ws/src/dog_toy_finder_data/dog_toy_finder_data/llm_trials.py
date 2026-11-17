"""Schema, metrics, and scoring for LLM comparison trials.

Two kinds of trial are compared with the same tooling:

- ``vision``: one row per vision request and model. The
  ``vision_trial_collector`` node writes these while ``VISION_MODELS`` holds
  a comma-separated list, so every model sees the same stereo photo.
- ``planning``: one row per prompt and model. ``rosa_agent --compare`` writes
  these, including the reply and the tools the model called.

Collectors write the automated columns (latency, model answer, tool calls) and
leave every ``gt_*`` ground-truth column empty. Fill those in a spreadsheet, or
supply them in the vision ask JSON / ROSA ``--gt-json`` file, then run
``llm_trials score``. Scoring fills the correctness columns and prints a
per-model summary.

Usage inside the ROS 2 container::

    ros2 run dog_toy_finder_data llm_trials score vision_trials.csv
    ros2 run dog_toy_finder_data llm_trials aggregate planning_trials.csv
    ros2 run dog_toy_finder_data llm_trials analyze vision_trials.csv
    ros2 run dog_toy_finder_data llm_trials template --kind vision \
        --models gemma4,minicpm-v4.5 --output vision_trials.csv
"""

import argparse
import csv
import json
import os
import sys
import time

KIND_VISION = "vision"
KIND_PLANNING = "planning"

# Automated columns first, then the blank ground-truth fields, then the
# metrics filled in by `score`. Keeping the order fixed makes the CSV easy to
# annotate in a spreadsheet and stable across runs.
VISION_FIELDS = [
    "trial_id", "kind", "run_id", "t", "ask_id", "target", "model", "method",
    "image",
    "latency_s", "error", "description", "target_visible", "position", "size",
    "distance_m", "obstacle_ahead", "path_clear",
    "gt_target_visible", "gt_position", "gt_size", "gt_distance_m",
    "gt_obstacle_ahead", "gt_notes",
    "visible_correct", "position_correct", "size_correct", "distance_error_m",
    "distance_within_tolerance", "obstacle_correct", "score",
]

PLANNING_FIELDS = [
    "trial_id", "kind", "run_id", "t", "model", "prompt", "reply", "elapsed_s",
    "reply_chars", "tool_calls", "error",
    "gt_expected_action", "gt_expected_args", "gt_success", "gt_notes",
    "action_mentioned", "args_mentioned", "score",
]

VISION_GT_FIELDS = [f for f in VISION_FIELDS if f.startswith("gt_")]
PLANNING_GT_FIELDS = [f for f in PLANNING_FIELDS if f.startswith("gt_")]
VISION_METRIC_FIELDS = [
    "visible_correct", "position_correct", "size_correct", "distance_error_m",
    "distance_within_tolerance", "obstacle_correct", "score",
]
PLANNING_METRIC_FIELDS = ["action_mentioned", "args_mentioned", "score"]

TRUE_WORDS = {"true", "1", "yes", "y", "visible", "present", "found"}
FALSE_WORDS = {"false", "0", "no", "n", "not visible", "absent", "missing",
               "none"}
NO_OBSTACLE_WORDS = {"", "none", "nothing", "no", "n/a", "na", "clear"}


def _as_bool(value):
    """Return True/False for common CSV spellings, or None when blank/unknown."""
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("", "unknown"):
        return None
    if text in TRUE_WORDS:
        return True
    if text in FALSE_WORDS:
        return False
    return None


def _as_float(value):
    """Return a finite float, or None when the value is missing/not a number."""
    try:
        number = float(str(value).strip())
    except (TypeError, ValueError):
        return None
    if number != number or number in (float("inf"), float("-inf")):
        return None
    return number


def _as_float_list(value):
    """Return finite numbers from a scalar, JSON array, or delimited string."""
    if isinstance(value, (list, tuple)):
        items = value
    else:
        text = str(value or "").strip()
        if not text:
            return []
        if text.startswith("[") and text.endswith("]"):
            try:
                parsed = json.loads(text)
            except ValueError:
                parsed = None
            items = parsed if isinstance(parsed, (list, tuple)) else \
                text.strip("[]").replace(";", ",").split(",")
        else:
            items = text.replace(";", ",").split(",")

    numbers = []
    for item in items:
        number = _as_float(item)
        if number is not None:
            numbers.append(number)
    return numbers


def _serialise_distance_truth(value):
    """Keep multiple ground-truth distances as JSON so CSV round-trips them."""
    if not isinstance(value, (list, tuple)):
        return value
    distances = [number for number in _as_float_list(value) if number > 0.0]
    return json.dumps(distances) if distances else ""


def _text(value):
    """Normalise free text for matching: lower case, single spaces, no edges."""
    return " ".join(str(value or "").strip().lower().split())


def _obstacle(value):
    """Normalise an obstacle name, mapping the many 'no obstacle' words to none."""
    text = _text(value).strip(".")
    return "none" if text in NO_OBSTACLE_WORDS else text


def _blank(fields):
    """Return a row dict with every requested field set to an empty string."""
    return {field: "" for field in fields}


def merged_ground_truth(ground_truth):
    """Map a ground-truth dict to the ``gt_*`` column names.

    Accepts either the short keys used in the vision ask JSON
    (``target_visible``, ``position``, ...) or the actual column names
    (``gt_target_visible``, ...). Unknown keys are ignored. Distance ground
    truth may be a number or a list of repeated measurements; lists are stored
    as JSON so they survive the CSV round trip.
    """
    if not isinstance(ground_truth, dict):
        return {}
    aliases = {
        "target_visible": "gt_target_visible",
        "visible": "gt_target_visible",
        "position": "gt_position",
        "size": "gt_size",
        "distance_m": "gt_distance_m",
        "distance": "gt_distance_m",
        "distances_m": "gt_distance_m",
        "distances": "gt_distance_m",
        "obstacle_ahead": "gt_obstacle_ahead",
        "notes": "gt_notes",
        "expected_action": "gt_expected_action",
        "action": "gt_expected_action",
        "expected_args": "gt_expected_args",
        "args": "gt_expected_args",
        "success": "gt_success",
    }
    result = {}
    for key, value in ground_truth.items():
        field = aliases.get(key, key)
        if field in VISION_GT_FIELDS or field in PLANNING_GT_FIELDS:
            if field == "gt_distance_m":
                value = _serialise_distance_truth(value)
            result[field] = value
    return result


def apply_ground_truth(row, ground_truth):
    """Copy recognised ground-truth values into an existing trial row."""
    row.update(merged_ground_truth(ground_truth))
    return row


def vision_metrics(row, distance_tolerance=0.25):
    """Compute vision correctness columns from a row's answers and ground truth.

    Returns blank metrics until ``gt_target_visible`` is filled. A blank
    metric means that check does not apply to this row (for example a distance
    when the target is not visible). When ``gt_distance_m`` contains multiple
    measurements, their mean is used as the ground-truth distance.
    """
    metrics = _blank(VISION_METRIC_FIELDS)
    gt_visible = _as_bool(row.get("gt_target_visible"))
    if gt_visible is None:
        return metrics

    model_visible = _as_bool(row.get("target_visible"))
    metrics["visible_correct"] = model_visible == gt_visible
    checks = [metrics["visible_correct"]]

    if not gt_visible:
        obstacle_truth = _text(row.get("gt_obstacle_ahead")).strip(".")
        if obstacle_truth:
            metrics["obstacle_correct"] = _obstacle(
                row.get("obstacle_ahead")) == _obstacle(obstacle_truth)
            checks.append(metrics["obstacle_correct"])
        metrics["score"] = round(sum(1 for c in checks if c) / len(checks), 3)
        return metrics

    gt_position = _text(row.get("gt_position"))
    if gt_position and gt_position != "none":
        metrics["position_correct"] = _text(row.get("position")) == gt_position
        checks.append(metrics["position_correct"])

    gt_size = _text(row.get("gt_size"))
    if gt_size and gt_size != "none":
        metrics["size_correct"] = _text(row.get("size")) == gt_size
        checks.append(metrics["size_correct"])

    gt_distances = [number for number in _as_float_list(
        row.get("gt_distance_m")) if number > 0.0]
    if gt_distances:
        gt_distance = sum(gt_distances) / len(gt_distances)
        model_distance = _as_float(row.get("distance_m"))
        if model_distance is not None:
            error = abs(model_distance - gt_distance)
            metrics["distance_error_m"] = round(error, 3)
            metrics["distance_within_tolerance"] = error <= distance_tolerance
            checks.append(metrics["distance_within_tolerance"])

    obstacle_truth = _text(row.get("gt_obstacle_ahead")).strip(".")
    if obstacle_truth:
        metrics["obstacle_correct"] = _obstacle(
            row.get("obstacle_ahead")) == _obstacle(obstacle_truth)
        checks.append(metrics["obstacle_correct"])

    metrics["score"] = round(sum(1 for c in checks if c) / len(checks), 3)
    return metrics


def planning_metrics(row):
    """Compute planning correctness columns from a reply and ground truth.

    ``action_mentioned`` and ``args_mentioned`` are text checks against the
    reply and captured tool calls, so they are cheap automatic signals rather
    than a judgement of the whole plan. ``gt_success`` overrides them when the
    operator records whether the trial actually succeeded.
    """
    metrics = _blank(PLANNING_METRIC_FIELDS)
    haystack = _text(row.get("reply")) + " " + _text(row.get("tool_calls"))

    expected_action = _text(row.get("gt_expected_action"))
    if expected_action:
        metrics["action_mentioned"] = expected_action in haystack

    expected_args = str(row.get("gt_expected_args") or "")
    arg_tokens = [_text(token) for token in expected_args.replace(";", ",").split(",")]
    arg_tokens = [token for token in arg_tokens if token]
    if arg_tokens:
        metrics["args_mentioned"] = all(token in haystack for token in arg_tokens)

    success = _as_bool(row.get("gt_success"))
    if success is not None:
        metrics["score"] = 1.0 if success else 0.0
    else:
        checks = [metrics["action_mentioned"], metrics["args_mentioned"]]
        checks = [check for check in checks if isinstance(check, bool)]
        metrics["score"] = round(sum(1 for c in checks if c) / len(checks), 3) \
            if checks else ""
    return metrics


def vision_row(result, entry, model, run_id, distance_tolerance=0.25,
               ground_truth=None):
    """Build one vision trial row from a /vision/result model entry."""
    row = _blank(VISION_FIELDS)
    ask_id = str(result.get("id", ""))
    row.update({
        "kind": KIND_VISION,
        "run_id": run_id,
        "t": round(time.time(), 3),
        "ask_id": ask_id,
        "target": result.get("target", ""),
        "model": entry.get("model") or model or "",
        "method": entry.get("method") or (
            "classical" if str(entry.get("model") or model).strip().lower()
            == "classical" else "ml"),
        "image": result.get("image", ""),
    })
    row["trial_id"] = "%s-%s-%s" % (run_id, ask_id, row["model"])
    for key in ("description", "target_visible", "position", "size",
                "distance_m", "obstacle_ahead", "path_clear", "error"):
        if key in entry:
            row[key] = entry[key]
    if "seconds" in entry:
        row["latency_s"] = entry["seconds"]
    apply_ground_truth(row, ground_truth)
    row.update(vision_metrics(row, distance_tolerance))
    return row


def planning_row(model, prompt, reply, elapsed_s, run_id, tool_calls=None,
                 error="", ground_truth=None):
    """Build one planning trial row from a ROSA comparison result."""
    row = _blank(PLANNING_FIELDS)
    row.update({
        "kind": KIND_PLANNING,
        "run_id": run_id,
        "t": round(time.time(), 3),
        "model": model,
        "prompt": prompt,
        "reply": reply,
        "elapsed_s": round(float(elapsed_s), 3),
        "reply_chars": len(str(reply or "")),
        "error": error,
    })
    if tool_calls:
        row["tool_calls"] = json.dumps(tool_calls)
    row["trial_id"] = "%s-%s-%s" % (
        run_id, model, int(time.time() * 1000))
    apply_ground_truth(row, ground_truth)
    row.update(planning_metrics(row))
    return row


def append_csv(path, row, fieldnames):
    """Append one row to a CSV, writing the header when the file is new."""
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    new_file = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, "a", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames,
                                extrasaction="ignore")
        if new_file:
            writer.writeheader()
        writer.writerow({field: row.get(field, "") for field in fieldnames})


def read_csv(path):
    """Read a trial CSV and return (rows, fieldnames)."""
    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        return list(reader), list(reader.fieldnames or [])


def write_csv(path, rows, fieldnames):
    """Write rows with a fixed column order."""
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames,
                                extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fieldnames})


def kind_of(rows, fieldnames=None):
    """Return the trial kind, inferring it from the columns when necessary."""
    for row in rows:
        kind = row.get("kind", "").strip().lower()
        if kind in (KIND_VISION, KIND_PLANNING):
            return kind
    fields = set(fieldnames or [])
    if "gt_expected_action" in fields or "tool_calls" in fields:
        return KIND_PLANNING
    return KIND_VISION


def score_rows(rows, distance_tolerance=0.25):
    """Fill and return the metric columns for every row in place."""
    kind = kind_of(rows)
    for row in rows:
        if kind == KIND_VISION:
            row.update(vision_metrics(row, distance_tolerance))
        else:
            row.update(planning_metrics(row))
    return rows


def _mean(values):
    """Mean of the numeric values, ignoring blanks; None when there are none."""
    numbers = []
    for value in values:
        number = _as_float(value)
        if number is not None:
            numbers.append(number)
    return round(sum(numbers) / len(numbers), 3) if numbers else None


def _rate(values):
    """Fraction of true values among booleans, ignoring blanks; None if empty."""
    checks = [_as_bool(value) for value in values]
    checks = [check for check in checks if check is not None]
    return round(sum(1 for check in checks if check) / len(checks), 3) \
        if checks else None


def aggregate(rows, distance_tolerance=0.25):
    """Return per-model aggregate metrics as a list of dicts."""
    kind = kind_of(rows)
    score_rows(rows, distance_tolerance)
    models = {}
    for row in rows:
        models.setdefault(row.get("model", ""), []).append(row)

    summaries = []
    for model, model_rows in sorted(models.items()):
        summary = {"model": model, "trials": len(model_rows)}
        if kind == KIND_VISION:
            summary.update({
                "mean_latency_s": _mean([r.get("latency_s") for r in model_rows]),
                "errors": sum(1 for r in model_rows if r.get("error")),
                "gt_rows": sum(1 for r in model_rows
                               if _as_bool(r.get("gt_target_visible")) is not None),
                "visible_accuracy": _rate(
                    [r.get("visible_correct") for r in model_rows]),
                "position_accuracy": _rate(
                    [r.get("position_correct") for r in model_rows]),
                "size_accuracy": _rate(
                    [r.get("size_correct") for r in model_rows]),
                "distance_mae_m": _mean(
                    [r.get("distance_error_m") for r in model_rows]),
                "obstacle_accuracy": _rate(
                    [r.get("obstacle_correct") for r in model_rows]),
                "mean_score": _mean([r.get("score") for r in model_rows]),
            })
        else:
            summary.update({
                "mean_elapsed_s": _mean([r.get("elapsed_s") for r in model_rows]),
                "errors": sum(1 for r in model_rows if r.get("error")),
                "gt_rows": sum(
                    1 for r in model_rows
                    if any(str(r.get(field, "")).strip()
                           for field in PLANNING_GT_FIELDS)),
                "action_mention_rate": _rate(
                    [r.get("action_mentioned") for r in model_rows]),
                "args_mention_rate": _rate(
                    [r.get("args_mentioned") for r in model_rows]),
                "mean_score": _mean([r.get("score") for r in model_rows]),
            })
        summaries.append(summary)
    return summaries


def print_summaries(summaries, as_json=False):
    """Print aggregate metrics as JSON or a compact human-readable block."""
    if as_json:
        print(json.dumps(summaries, indent=2))
        return
    if not summaries:
        print("No rows to summarise.")
        return
    for summary in summaries:
        print("\n--- model: %s ---" % summary["model"])
        for key, value in summary.items():
            if key != "model":
                print("  %-22s %s" % (key + ":", value))


def write_template(path, kind, models=None, run_id="run1", target="red ball",
                   prompt=""):
    """Write an empty trial CSV with ground-truth columns ready to fill in."""
    fields = VISION_FIELDS if kind == KIND_VISION else PLANNING_FIELDS
    rows = []
    for model in (models or [""]):
        row = _blank(fields)
        row.update({"kind": kind, "run_id": run_id, "model": model,
                    "t": round(time.time(), 3)})
        if kind == KIND_VISION:
            row["target"] = target
            row["trial_id"] = "%s-%s-template" % (run_id, model)
        else:
            row["prompt"] = prompt
            row["trial_id"] = "%s-%s-template" % (run_id, model)
        rows.append(row)
    write_csv(path, rows, fields)
    return path


def _parser():
    """Build the command-line parser for the llm_trials tool."""
    parser = argparse.ArgumentParser(
        description="Score and summarise LLM vision/planning comparison trials")
    sub = parser.add_subparsers(dest="command", required=True)

    score = sub.add_parser("score", help="fill metric columns and print a summary")
    score.add_argument("csv", help="trial CSV written by a collector or ROSA")
    score.add_argument("-o", "--output", default=None,
                       help="output CSV (default: <name>_scored.csv)")
    score.add_argument("--tolerance", type=float, default=0.25,
                       help="distance tolerance in metres (default: 0.25)")
    score.add_argument("--json", action="store_true",
                       help="print the per-model summary as JSON")

    aggregate_parser = sub.add_parser(
        "aggregate", help="print per-model metrics without writing a file")
    aggregate_parser.add_argument("csv", help="trial CSV")
    aggregate_parser.add_argument("--tolerance", type=float, default=0.25,
                                  help="distance tolerance in metres")
    aggregate_parser.add_argument("--json", action="store_true",
                                  help="print the summary as JSON")

    analysis = sub.add_parser(
        "analyze", help="descriptive stats and paired model comparisons")
    analysis.add_argument("csv", help="trial CSV")
    analysis.add_argument("-o", "--output", default=None,
                          help="JSON report (default: <name>_analysis.json)")
    analysis.add_argument("--csv-output", default=None,
                          help="optional tidy CSV version of the analysis")
    analysis.add_argument("--confidence", type=float, default=0.95,
                          help="confidence level for intervals (default: 0.95)")
    analysis.add_argument("--tolerance", type=float, default=0.25,
                          help="distance tolerance in metres when scoring")
    analysis.add_argument("--pair-field", default=None,
                          help="comma-separated fields matching rows across "
                               "models (default: ask_id for vision, prompt "
                               "for planning)")
    analysis.add_argument("--json", action="store_true",
                          help="print the full analysis as JSON")

    template = sub.add_parser(
        "template", help="create a CSV with empty ground-truth columns")
    template.add_argument("--kind", choices=[KIND_VISION, KIND_PLANNING],
                          required=True, help="trial kind")
    template.add_argument("-o", "--output", required=True, help="CSV path")
    template.add_argument("--models", default="",
                          help="comma-separated model names (one row each)")
    template.add_argument("--run-id", default="run1", help="run label")
    template.add_argument("--target", default="red ball",
                          help="vision target description")
    template.add_argument("--prompt", default="", help="planning prompt")
    return parser


def main(argv=None):
    """Run the llm_trials command-line interface."""
    args = _parser().parse_args(argv)
    if args.command == "template":
        models = [m.strip() for m in args.models.split(",") if m.strip()]
        path = write_template(args.output, args.kind, models,
                              run_id=args.run_id, target=args.target,
                              prompt=args.prompt)
        print("Wrote template with %d row(s) to %s" % (len(models) or 1, path))
        return 0

    if args.command == "analyze":
        try:
            from . import trial_stats
        except ImportError:
            import trial_stats
        rows, _ = read_csv(args.csv)
        if not rows:
            print("No rows in %s" % args.csv)
            return 1
        analysis = trial_stats.analyze(
            rows, confidence=args.confidence, pair_field=args.pair_field,
            distance_tolerance=args.tolerance)
        output = args.output
        if output is None:
            stem, extension = os.path.splitext(args.csv)
            output = stem + "_analysis" + (extension or ".json")
        os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
        with open(output, "w") as handle:
            json.dump(analysis, handle, indent=2)
        if args.csv_output:
            trial_stats.write_analysis_csv(args.csv_output, analysis)
        print("Wrote statistical analysis to %s" % output)
        if args.json:
            print(json.dumps(analysis, indent=2))
        else:
            trial_stats.print_summary(analysis)
        return 0

    rows, fields = read_csv(args.csv)
    if not rows:
        print("No rows in %s" % args.csv)
        return 1
    kind = kind_of(rows, fields)
    score_rows(rows, args.tolerance)
    summaries = aggregate(rows, args.tolerance)
    if args.command == "score":
        output = args.output
        if output is None:
            stem, extension = os.path.splitext(args.csv)
            output = stem + "_scored" + (extension or ".csv")
        ordered = VISION_FIELDS if kind == KIND_VISION else PLANNING_FIELDS
        write_csv(output, rows, ordered)
        print("Scored %d %s trial(s) -> %s" % (len(rows), kind, output))
    print_summaries(summaries, as_json=args.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
