"""Command-line entry point for the Dog Toy Finder ROSA agent.

Run from the software/rosa_agent folder after sourcing the ROS 2 workspace:
    python3 -m rosa_agent
"""

from .agent import build_agent, ollama_model_name
from .robot import close_robot, get_robot


def main() -> None:
    """Start the agent, handle requests until the user quits, then clean up."""
    get_robot()
    agent = build_agent()
    print(
        "Dog Toy Finder agent ready (model: %s). Type 'quit' to exit."
        % ollama_model_name()
    )
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
            except Exception as error:  # keep the CLI alive on tool/LLM errors
                get_robot().mover.stop()
                print("ROSA error: %s. The robot was stopped." % error)
    finally:
        close_robot()


if __name__ == "__main__":
    main()
