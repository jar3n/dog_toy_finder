"""Command-line entry point for the Dog Toy Finder ROSA agent.

Supports comparing several LLMs on the same request:

    python3 -m rosa_agent --model llama3.2
    python3 -m rosa_agent --compare llama3.2,mistral,qwen2.5
    python3 -m rosa_agent --compare llama3.2,mistral --prompt "Drive forward 1 meter."

In compare mode the same prompt is run through every model and the reply plus
the wall-clock time are printed for each, so the models can be compared.
"""

import argparse
import time

from .agent import build_agent, ollama_model_name
from .robot import close_robot, get_robot
from .vision import close_vision


def build_parser():
    parser = argparse.ArgumentParser(description="Dog Toy Finder ROSA agent")
    parser.add_argument("--model", default=None,
                        help="single Ollama model to use (default: OLLAMA_MODEL)")
    parser.add_argument("--compare", default=None,
                        help="comma-separated list of models to compare on one prompt")
    parser.add_argument("--prompt", default=None,
                        help="run a single prompt and exit (useful with --compare)")
    return parser


def run_single(model):
    get_robot()
    agent = build_agent(model=model)
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
            try:
                print("ROSA> %s" % agent.invoke(request))
            except KeyboardInterrupt:
                get_robot().mover.stop()
                print("\nInterrupted. The robot was stopped.")
            except Exception as error:
                get_robot().mover.stop()
                print("ROSA error: %s. The robot was stopped." % error)
    finally:
        close_robot()
        close_vision()


def run_compare(models, prompt):
    get_robot()
    for model in models:
        print(f"\n===== model: {model} =====")
        agent = build_agent(model=model)
        start = time.time()
        try:
            reply = agent.invoke(prompt)
            elapsed = time.time() - start
            print(f"ROSA> {reply}")
            print(f"[{elapsed:.2f} s]")
        except Exception as error:
            elapsed = time.time() - start
            print(f"ROSA error: {error}")
            print(f"[{elapsed:.2f} s]")
    close_robot()
    close_vision()


def main():
    args = build_parser().parse_args()
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
        run_compare(models, prompt)
        return
    run_single(args.model)


if __name__ == "__main__":
    main()
