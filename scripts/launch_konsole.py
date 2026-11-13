"""Launch the Dog Toy Finder stack in one Konsole window with split panes.

Reads a tab-separated commands file (one ``NAME<TAB>COMMAND`` per line) and
opens a single Konsole window split into one pane per command, running each
command in its pane. Uses Konsole's D-Bus interface, so the security-sensitive
API must be enabled: ``EnableSecuritySensitiveDBusAPI=true`` under
``[KonsoleWindow]`` in ``~/.config/konsolerc``.

Usage:
    python3 launch_konsole.py COMMANDS_FILE [WORKDIR]
"""
import json
import signal
import os
import subprocess
import sys
import tempfile
import time


def qdbus(svc, path, method, *args):
    cmd = ["qdbus-qt6", svc, path, method] + [str(a) for a in args]
    return subprocess.check_output(cmd, text=True).strip()


def build_layout(n):
    """Build a 2-column split layout JSON for n panes."""
    rows = (n + 1) // 2
    idx = 0
    row_widgets = []
    for r in range(rows):
        leaves = []
        for c in range(2):
            if idx < n:
                leaves.append({"SessionRestoreId": 0})
                idx += 1
        if len(leaves) == 1:
            row_widgets.append(leaves[0])
        else:
            row_widgets.append({"Orientation": "Horizontal", "Widgets": leaves})
    if len(row_widgets) == 1:
        return row_widgets[0]
    return {"Orientation": "Vertical", "Widgets": row_widgets}


def main():
    if len(sys.argv) < 2:
        print("usage: launch_konsole.py COMMANDS_FILE [WORKDIR]", file=sys.stderr)
        sys.exit(2)
    commands_file = sys.argv[1]
    workdir = sys.argv[2] if len(sys.argv) > 2 else os.getcwd()

    entries = []
    with open(commands_file) as f:
        for line in f:
            line = line.rstrip("\n")
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            parts = line.split("\t", 1)
            if len(parts) != 2:
                continue
            entries.append((parts[0].strip(), parts[1].strip()))

    if not entries:
        print("no commands to launch", file=sys.stderr)
        sys.exit(1)

    n = len(entries)
    layout = build_layout(n)

    fd, layout_path = tempfile.mkstemp(suffix=".json", prefix="konsole_layout_")
    with os.fdopen(fd, "w") as f:
        json.dump(layout, f, indent=2)

    konsole = subprocess.Popen(
        ["konsole", "--layout", layout_path, "--workdir", workdir],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    time.sleep(4)

    services = subprocess.check_output(["qdbus-qt6"], text=True).splitlines()
    svc_candidates = [s.strip() for s in services if "konsole" in s]
    if not svc_candidates:
        print("konsole D-Bus service not found", file=sys.stderr)
        sys.exit(1)
    svc = svc_candidates[-1]

    sess_out = subprocess.check_output(
        ["qdbus-qt6", svc, "/Windows/1", "org.kde.konsole.Window.sessionList"],
        text=True).split()
    session_ids = [int(x) for x in sess_out]
    creation_order = list(reversed(session_ids))

    if len(creation_order) < n:
        print("warning: fewer sessions (%d) than commands (%d)"
              % (len(creation_order), n), file=sys.stderr)

    for (name, cmd), sid in zip(entries, creation_order):
        full = cmd if cmd.lstrip().startswith("cd ") else "cd %s && %s" % (workdir, cmd)
        subprocess.run(
            ["qdbus-qt6", svc, "/Sessions/%d" % sid,
             "org.kde.konsole.Session.runCommand", full],
            check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        print("launched [%s] in session %d" % (name, sid), flush=True)

    print("Konsole launched with %d panes. Close the window to stop." % n, flush=True)

    stopping = {"flag": False}

    def _stop(signum, frame):
        stopping["flag"] = True
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    def _cleanup():
        try:
            konsole.terminate()
            konsole.wait(timeout=3)
        except Exception:
            try:
                konsole.kill()
            except Exception:
                pass
        # Stop compose services and remove per-component run containers.
        for compose in (os.path.join(workdir, "docker", "docker-compose.yml"),
                        os.path.join(workdir, "..", "docker", "docker-compose.yml")):
            compose = os.path.normpath(compose)
            if os.path.isfile(compose):
                try:
                    subprocess.run(
                        ["docker", "compose", "-f", compose, "down"],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                        timeout=30)
                except Exception:
                    pass
                break
        for filt in ("name=docker-ros2-run", "name=micro-ros-agent"):
            try:
                ids = subprocess.check_output(
                    ["docker", "ps", "-q", "--filter", filt],
                    text=True, timeout=15).split()
                if ids:
                    subprocess.run(["docker", "rm", "-f"] + ids,
                                   stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL, timeout=30)
            except Exception:
                pass

    try:
        konsole.wait()
    except KeyboardInterrupt:
        pass
    finally:
        _cleanup()


if __name__ == "__main__":
    main()
