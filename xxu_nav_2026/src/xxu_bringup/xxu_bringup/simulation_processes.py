"""Stop only simulation/mapping launch trees owned by a selected workspace."""

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import signal
import time


LAUNCHES = {
    "xxu_bringup": {"gazebo.launch.py", "simulation.launch.py", "rm_sim_gazebo.launch.py",
                    "rm_sim_navigation.launch.py", "rmul_navigation.launch.py"},
    "xxu_slam_toolbox": {"mapping.launch.py", "autonomous_mapping.launch.py"},
}


@dataclass(frozen=True)
class Process:
    pid: int
    parent: int
    started: str
    args: tuple
    cwd: Path | None


def snapshot(proc_root=Path("/proc")):
    processes = {}
    for folder in proc_root.iterdir():
        if not folder.name.isdecimal():
            continue
        try:
            if folder.stat().st_uid != os.getuid():
                continue
            stat = (folder / "stat").read_text().rsplit(")", 1)[1].split()
            if stat[0] == "Z":
                continue
            args = tuple(part.decode(errors="replace") for part in
                         (folder / "cmdline").read_bytes().split(b"\0") if part)
            try:
                cwd = (folder / "cwd").resolve(strict=True)
            except OSError:
                cwd = None
            process = Process(int(folder.name), int(stat[1]), stat[19], args, cwd)
            processes[process.pid] = process
        except (OSError, IndexError, ValueError):
            continue
    return processes


def launch_owner(process, workspace):
    if process.cwd is None or not process.cwd.is_relative_to(workspace):
        return False
    for i, arg in enumerate(process.args):
        if Path(arg).name != "ros2" or process.args[i + 1:i + 2] != ("launch",):
            continue
        tail = process.args[i + 2:]
        if len(tail) >= 2 and tail[1] in LAUNCHES.get(tail[0], set()):
            return True
    return False


def select_trees(processes, workspace, caller=None):
    # Never target this invocation's ancestors, even when it was itself
    # started under a launch process.
    protected = set()
    while caller in processes and caller not in protected:
        protected.add(caller)
        caller = processes[caller].parent
    roots = {pid for pid, process in processes.items()
             if pid not in protected and launch_owner(process, workspace)}
    selected = set(roots)
    while True:
        children = {pid for pid, process in processes.items()
                    if process.parent in selected and pid not in protected}
        if children <= selected:
            return roots, selected
        selected |= children


def signal_process(process, signum):
    # PID reuse must never turn a cleanup into a signal to an unrelated node.
    try:
        stat = Path(f"/proc/{process.pid}/stat").read_text().rsplit(")", 1)[1].split()
        if stat[19] == process.started and stat[0] != "Z":
            os.kill(process.pid, signum)
    except (ProcessLookupError, PermissionError, FileNotFoundError):
        pass


def default_workspace():
    for source in (Path(__file__).resolve(), Path.cwd()):
        for parent in (source, *source.parents):
            if (parent / "src/xxu_bringup/package.xml").is_file():
                return parent
    raise ValueError("Cannot identify workspace; pass --workspace explicitly")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path)
    parser.add_argument("--dry-run", action="store_true", help="List selected PIDs without sending signals")
    args = parser.parse_args(argv)
    workspace = args.workspace.resolve() if args.workspace else default_workspace()
    if not (workspace / "src/xxu_bringup/package.xml").is_file():
        parser.error("workspace must contain src/xxu_bringup/package.xml")
    processes = snapshot()
    roots, selected = select_trees(processes, workspace, os.getpid())
    print(f"Workspace {workspace}: {len(roots)} simulation/mapping launches, {len(selected)} processes")
    if args.dry_run:
        print("Selected PIDs:", " ".join(map(str, sorted(selected))) or "none")
        return 0
    if not selected:
        return 0
    # Let ROS launch propagate SIGINT and run map-saving/cleanup callbacks.
    for pid in roots:
        signal_process(processes[pid], signal.SIGINT)
    pending = set(selected)
    for signum, timeout in ((None, 12), (signal.SIGTERM, 4), (signal.SIGKILL, 1)):
        if signum is not None:
            for pid in pending:
                signal_process(processes[pid], signum)
        deadline = time.monotonic() + timeout
        while pending and time.monotonic() < deadline:
            current = snapshot()
            # Include new descendants launched during orderly shutdown, but
            # retain the original identity for every already tracked PID.
            for pid, process in current.items():
                if process.parent in pending and pid not in processes:
                    processes[pid] = process
                    pending.add(pid)
            pending = {pid for pid in pending if pid in current
                       and current[pid].started == processes[pid].started}
            if pending:
                time.sleep(0.1)
    if pending:
        print("Processes still running:", " ".join(map(str, sorted(pending))))
        return 1
    print("Selected simulation/mapping launches stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
