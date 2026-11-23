"""Descriptive and paired statistics for LLM comparison trials.

This module analyses trial CSV rows produced by ``llm_trials`` collectors after
ground truth has been filled in. It has no third-party dependencies:

- descriptive statistics per model and metric (mean, median, sample standard
  deviation, standard error, and t-based confidence interval);
- paired comparisons between models on matched asks/prompts (mean difference,
  confidence interval, two-sided paired t-test, Cohen's dz, and Holm-adjusted
  p-values across all comparisons).

Vision rows are matched by ``ask_id``; planning rows are matched by ``prompt``.
Use ``--pair-field`` on the CLI when another field identifies the shared trial.
"""

import csv
import json
import math
import os

try:
    from . import llm_trials
except ImportError:  # direct import from the package directory in tests
    import llm_trials


VISION_METRICS = [
    "latency_s", "score", "visible_correct", "position_correct",
    "size_correct", "distance_within_tolerance", "distance_error_m",
    "bearing_within_tolerance", "bearing_error_deg", "obstacle_correct",
]
PLANNING_METRICS = [
    "elapsed_s", "score", "reply_chars", "action_mentioned", "args_mentioned",
]

ANALYSIS_CSV_FIELDS = [
    "scope", "metric", "model", "model_a", "model_b", "n", "mean", "median",
    "std", "sem", "ci_low", "ci_high", "min", "max", "mean_diff", "std_diff",
    "t", "p_value", "p_value_holm", "effect_dz", "a_wins", "ties", "b_wins",
]


# --- Numerical helpers (stdlib only) -------------------------------------


def _mean(values):
    return sum(values) / len(values) if values else None


def _median(values):
    if not values:
        return None
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) / 2.0


def _sample_std(values):
    if len(values) < 2:
        return None
    average = _mean(values)
    variance = sum((value - average) ** 2 for value in values) / (len(values) - 1)
    return math.sqrt(variance)


def _betacf(a, b, x):
    """Continued fraction for the regularized incomplete beta function."""
    max_iterations = 300
    epsilon = 3.0e-14
    tiny = 1.0e-300

    qab = a + b
    qap = a + 1.0
    qam = a - 1.0
    c = 1.0
    d = 1.0 - qab * x / qap
    if abs(d) < tiny:
        d = tiny
    d = 1.0 / d
    result = d

    for m in range(1, max_iterations + 1):
        m2 = 2 * m
        aa = m * (b - m) * x / ((qam + m2) * (a + m2))
        d = 1.0 + aa * d
        if abs(d) < tiny:
            d = tiny
        c = 1.0 + aa / c
        if abs(c) < tiny:
            c = tiny
        d = 1.0 / d
        result *= d * c

        aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2))
        d = 1.0 + aa * d
        if abs(d) < tiny:
            d = tiny
        c = 1.0 + aa / c
        if abs(c) < tiny:
            c = tiny
        d = 1.0 / d
        delta = d * c
        result *= delta
        if abs(delta - 1.0) < epsilon:
            break
    return result


def _betai(a, b, x):
    """Regularized incomplete beta function I_x(a, b)."""
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    coefficient = math.exp(
        math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b)
        + a * math.log(x) + b * math.log1p(-x))
    if x < (a + 1.0) / (a + b + 2.0):
        return coefficient * _betacf(a, b, x) / a
    return 1.0 - coefficient * _betacf(b, a, 1.0 - x) / b


def t_two_sided_p(t_value, degrees_of_freedom):
    """Two-sided p-value for a Student's t statistic."""
    if degrees_of_freedom <= 0:
        return None
    if t_value is None:
        return None
    if t_value == 0:
        return 1.0
    x = degrees_of_freedom / (degrees_of_freedom + t_value * t_value)
    return min(1.0, max(0.0, _betai(degrees_of_freedom / 2.0, 0.5, x)))


def t_critical(degrees_of_freedom, confidence=0.95):
    """Positive t quantile for a two-sided confidence interval."""
    if degrees_of_freedom <= 0:
        return None
    alpha = 1.0 - confidence
    low, high = 0.0, 1.0e6
    for _ in range(200):
        middle = (low + high) / 2.0
        if t_two_sided_p(middle, degrees_of_freedom) > alpha:
            low = middle
        else:
            high = middle
    return (low + high) / 2.0


def describe(values, confidence=0.95):
    """Return descriptive statistics for a list of numeric observations."""
    numbers = [float(value) for value in values if value is not None]
    if not numbers:
        return None

    average = _mean(numbers)
    standard_deviation = _sample_std(numbers)
    standard_error = standard_deviation / math.sqrt(len(numbers)) \
        if standard_deviation is not None else 0.0
    critical = t_critical(len(numbers) - 1, confidence) \
        if standard_error is not None else None
    if critical is None:
        ci_low = ci_high = None
    else:
        ci_low = average - critical * standard_error
        ci_high = average + critical * standard_error

    return {
        "n": len(numbers),
        "mean": average,
        "median": _median(numbers),
        "std": standard_deviation,
        "sem": standard_error,
        "ci_low": ci_low,
        "ci_high": ci_high,
        "min": min(numbers),
        "max": max(numbers),
    }


def compare_paired(differences, confidence=0.95):
    """Return paired statistics for differences (model_a minus model_b)."""
    summary = describe(differences, confidence)
    if summary is None:
        return None

    standard_deviation = summary["std"]
    standard_error = summary["sem"]
    t_value = summary["mean"] / standard_error \
        if standard_error and standard_error > 0.0 else None
    p_value = t_two_sided_p(t_value, summary["n"] - 1) \
        if t_value is not None else None
    effect_size = summary["mean"] / standard_deviation \
        if standard_deviation and standard_deviation > 0.0 else None

    return {
        "n": summary["n"],
        "mean_diff": summary["mean"],
        "std_diff": standard_deviation,
        "sem": standard_error,
        "ci_low": summary["ci_low"],
        "ci_high": summary["ci_high"],
        "t": t_value,
        "p_value": p_value,
        "p_value_holm": p_value,
        "effect_dz": effect_size,
        "a_wins": sum(1 for value in differences if value > 0.0),
        "ties": sum(1 for value in differences if value == 0.0),
        "b_wins": sum(1 for value in differences if value < 0.0),
    }


def adjust_holm(comparisons):
    """Apply Holm's step-down correction to paired-comparison p-values."""
    for comparison in comparisons:
        comparison["p_value_holm"] = comparison.get("p_value")

    candidates = [item for item in comparisons if item.get("p_value") is not None]
    candidates.sort(key=lambda item: item["p_value"])
    count = len(candidates)
    previous = 0.0
    for rank, comparison in enumerate(candidates, 1):
        adjusted = min(1.0, max(
            previous, (count - rank + 1) * comparison["p_value"]))
        comparison["p_value_holm"] = adjusted
        previous = adjusted


def _significant(value):
    """Round to six significant digits, preserving small p-values."""
    if value is None or isinstance(value, bool):
        return value
    if isinstance(value, float):
        if math.isnan(value) or math.isinf(value):
            return None
        return float("%.6g" % value)
    if isinstance(value, dict):
        return {key: _significant(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_significant(item) for item in value]
    return value


# --- Trial analysis -------------------------------------------------------


def _metric_value(row, metric):
    """Return one numeric metric from a row, or None when it is not present."""
    raw = row.get(metric)
    if raw is None or str(raw).strip() == "":
        return None
    boolean = llm_trials._as_bool(raw)
    if boolean is not None:
        return 1.0 if boolean else 0.0
    return llm_trials._as_float(raw)


def _pair_fields(kind, pair_field):
    if pair_field:
        fields = [field.strip() for field in pair_field.split(",") if field.strip()]
        if fields:
            return fields
    if kind == llm_trials.KIND_VISION:
        return ["run_id", "ask_id"]
    return ["run_id", "prompt"]


def _pair_key(row, fields):
    return tuple(str(row.get(field, "") or "").strip() for field in fields)


def analyze(rows, confidence=0.95, pair_field=None, distance_tolerance=0.25,
            bearing_tolerance=5.0):
    """Build descriptive statistics and paired comparisons for trial rows.

    Rows are scored in place so analysis works on raw or already-scored CSVs.
    """
    if not 0.0 < confidence < 1.0:
        raise ValueError("confidence must be between 0 and 1")
    if not rows:
        return {
            "kind": None,
            "confidence": confidence,
            "metrics": [],
            "models": [],
            "descriptives": [],
            "comparisons": [],
        }

    kind = llm_trials.kind_of(rows)
    llm_trials.score_rows(rows, distance_tolerance, bearing_tolerance)
    metrics = VISION_METRICS if kind == llm_trials.KIND_VISION \
        else PLANNING_METRICS
    fields = _pair_fields(kind, pair_field)
    models = sorted({str(row.get("model", "")).strip() for row in rows
                     if str(row.get("model", "")).strip()})

    descriptives = []
    for model in models:
        model_rows = [row for row in rows if str(row.get("model", "")).strip() == model]
        for metric in metrics:
            values = [_metric_value(row, metric) for row in model_rows]
            summary = describe(values, confidence)
            if summary is None:
                continue
            descriptives.append({
                "scope": "descriptive",
                "metric": metric,
                "model": model,
                **summary,
            })

    paired = {}
    for row in rows:
        model = str(row.get("model", "")).strip()
        key = _pair_key(row, fields)
        if not model or not key[-1]:
            continue
        paired.setdefault(key, {})[model] = row

    comparisons = []
    for index, model_a in enumerate(models):
        for model_b in models[index + 1:]:
            for metric in metrics:
                differences = []
                for model_rows in paired.values():
                    if model_a not in model_rows or model_b not in model_rows:
                        continue
                    value_a = _metric_value(model_rows[model_a], metric)
                    value_b = _metric_value(model_rows[model_b], metric)
                    if value_a is not None and value_b is not None:
                        differences.append(value_a - value_b)
                summary = compare_paired(differences, confidence)
                if summary is None:
                    continue
                comparisons.append({
                    "scope": "comparison",
                    "metric": metric,
                    "model_a": model_a,
                    "model_b": model_b,
                    **summary,
                })

    adjust_holm(comparisons)
    analysis = {
        "kind": kind,
        "confidence": confidence,
        "pair_field": ",".join(fields),
        "metrics": metrics,
        "models": models,
        "descriptives": descriptives,
        "comparisons": comparisons,
    }
    return _significant(analysis)


def write_analysis_csv(path, analysis):
    """Write descriptive and comparison rows to one tidy CSV."""
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=ANALYSIS_CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in analysis.get("descriptives", []) \
                + analysis.get("comparisons", []):
            writer.writerow({field: row.get(field, "") for field in
                             ANALYSIS_CSV_FIELDS})
    return path


def print_summary(analysis):
    """Print a compact human-readable statistical report."""
    print("Trial kind: %s" % (analysis.get("kind") or "unknown"))
    print("Confidence interval: %.1f%%" % (
        100.0 * analysis.get("confidence", 0.95)))
    print("Matched on: %s" % analysis.get("pair_field", ""))

    print("\n--- Descriptive statistics ---")
    print("%-10s %-24s %5s %9s %9s %9s %18s" % (
        "model", "metric", "n", "mean", "median", "std", "95% CI"))
    for row in analysis.get("descriptives", []):
        ci = "%s .. %s" % (row.get("ci_low", ""), row.get("ci_high", ""))
        print("%-10s %-24s %5s %9s %9s %9s %18s" % (
            row.get("model", ""), row.get("metric", ""), row.get("n", ""),
            row.get("mean", ""), row.get("median", ""), row.get("std", ""),
            ci))

    print("\n--- Paired comparisons (model_a - model_b) ---")
    print("%-22s %-10s %-10s %5s %10s %11s %11s" % (
        "metric", "model_a", "model_b", "n", "mean_diff", "p", "p_holm"))
    comparisons = analysis.get("comparisons", [])
    if not comparisons:
        print("No matched pairs with more than one model were found.")
    for row in comparisons:
        print("%-22s %-10s %-10s %5s %10s %11s %11s" % (
            row.get("metric", ""), row.get("model_a", ""),
            row.get("model_b", ""), row.get("n", ""),
            row.get("mean_diff", ""), row.get("p_value", ""),
            row.get("p_value_holm", "")))


def main(argv=None):
    """Standalone entry point (normally invoked through ``llm_trials``)."""
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv")
    parser.add_argument("-o", "--output", required=True,
                        help="JSON analysis output path")
    parser.add_argument("--csv-output", default=None,
                        help="optional tidy CSV output path")
    parser.add_argument("--confidence", type=float, default=0.95)
    parser.add_argument("--pair-field", default=None)
    parser.add_argument("--tolerance", type=float, default=0.25)
    parser.add_argument("--bearing-tolerance", type=float, default=5.0)
    args = parser.parse_args(argv)

    with open(args.csv, newline="") as handle:
        rows = list(csv.DictReader(handle))
    analysis = analyze(rows, confidence=args.confidence,
                       pair_field=args.pair_field,
                       distance_tolerance=args.tolerance,
                       bearing_tolerance=args.bearing_tolerance)
    with open(args.output, "w") as handle:
        json.dump(analysis, handle, indent=2)
    if args.csv_output:
        write_analysis_csv(args.csv_output, analysis)
    print_summary(analysis)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
