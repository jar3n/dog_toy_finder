"""Command-line entry point for the Dog Toy Finder ROSA agent.

Supports comparing several LLMs on the same request:

    python3 -m rosa_agent --model llama3.2
    python3 -m rosa_agent --compare llama3.2,mistral,qwen2.5
    python3 -m rosa_agent --compare llama3.2,mistral --prompt "Drive forward 1 meter."

In compare mode the same prompt is run through every model and the reply plus
the wall-clock time are printed for each, so the models can be compared.

Pass ``--output-csv`` to also append one row per prompt and model to the shared
planning trial schema (see ``dog_toy_finder_data.llm_trials``). The row records
the reply, latency, and every tool the model called; the ``gt_*`` ground-truth
columns stay blank so they can be filled later, or ``--gt-json`` can supply
them up front.
"""

import argparse
import sys
import time

from . import trial_log
from .agent import (
    build_agent,
    invoke_with_events,
    ollama_model_name,
    unload_model,
)
from .robot import close_robot, get_robot
from .vision import close_vision


def _unload(model):
    """Evict a model after its comparison turn so the next gets all the VRAM."""
    if not unload_model(model):
        print("warning: could not unload %s; it may still hold VRAM" % model,
              file=sys.stderr)


def build_parser():
    parser = argparse.ArgumentParser(description="Dog Toy Finder ROSA agent")
    parser.add_argument("--model", default=None,
                        help="single Ollama model to use (default: OLLAMA_MODEL)")
    parser.add_argument("--compare", default=None,
                        help="comma-separated list of models to compare on one prompt")
    parser.add_argument("--prompt", default=None,
                        help="run a single prompt and exit (useful with --compare)")
    parser.add_argument("--output-csv", default=None,
                        help="append planning trials to this CSV (LLM comparison)")
    parser.add_argument("--run-id", default="plan1",
                        help="label stored in the planning trial CSV")
    parser.add_argument("--gt-json", default=None,
                        help="optional ground-truth JSON for the planning trial CSV")
    return parser


def _record(output_csv, run_id, model, prompt, reply, elapsed, calls, error,
            ground_truth):
    """Append one planning row when an output CSV was requested."""
    if not output_csv:
        return
    trial_log.record_planning_trial(
        output_csv, model, prompt, reply, elapsed, run_id,
        tool_calls=calls, error=error,
        ground_truth=trial_log.ground_truth_for(ground_truth, model, prompt))


def run_single(model, output_csv, run_id, ground_truth):
    get_robot()
    agent = build_agent(model=model, streaming=True)
    chosen = model if model else ollama_model_name()
    print(f"Dog Toy Finder agent ready (model: {chosen}). Type 'quit' to exit.")
    try:
        while True:
            try:
                request = input("You> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if request.lower() in ("quit", "exit"):
                break
            if not request:
                continue
            start = time.time()
            try:
                reply, calls, error = invoke_with_events(agent, request)
            except KeyboardInterrupt:
                get_robot().mover.stop()
                print("\nInterrupted. The robot was stopped.")
                continue
            except Exception as failure:
                get_robot().mover.stop()
                reply, calls, error = "", [], str(failure)
            elapsed = time.time() - start
            if error:
                get_robot().mover.stop()
                print("ROSA error: %s. The robot was stopped." % error)
            else:
                print("ROSA> %s" % reply)
            _record(output_csv, run_id, chosen, request, reply, elapsed, calls,
                    error, ground_truth)
    finally:
        close_robot()
        close_vision()


def run_compare(models, prompt, output_csv, run_id, ground_truth):
    get_robot()
    for model in models:
        print(f"\n===== model: {model} =====")
        agent = build_agent(model=model, streaming=True)
        start = time.time()
        try:
            reply, calls, error = invoke_with_events(agent, prompt)
        except Exception as failure:
            reply, calls, error = "", [], str(failure)
        elapsed = time.time() - start
        if error:
            print(f"ROSA error: {error}")
        else:
            print(f"ROSA> {reply}")
        print(f"[{elapsed:.2f} s]")
        _unload(model)
        _record(output_csv, run_id, model, prompt, reply, elapsed, calls, error,
                ground_truth)
    close_robot()
    close_vision()


def main():
    args = build_parser().parse_args()
    try:
        ground_truth = trial_log.load_ground_truth(args.gt_json)
    except (OSError, ValueError) as error:
        print("Could not read --gt-json: %s" % error)
        return 2
    models = [m.strip() for m in args.compare.split(",")] if args.compare else None
    if models:
        prompt = args.prompt
        if prompt is None:
            try:
                prompt = input("Prompt> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                return
        if not prompt:
            print("No prompt given.")
            return
        run_compare(models, prompt, args.output_csv, args.run_id, ground_truth)
        return
    run_single(args.model, args.output_csv, args.run_id, ground_truth)


if __name__ == "__main__":
    main()
