"""Subprocess wrapper around ``npm run aerodb -- --config cfg.json --out dir``."""

from __future__ import annotations

import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Mapping, Optional, Sequence, Union

PathLike = Union[str, os.PathLike]

EXIT_USAGE = 2
EXIT_INTERRUPTED = 130


class AeroDbError(RuntimeError):
    """The CLI exited nonzero. ``returncode`` and ``stderr`` carry the details."""

    def __init__(self, message: str, returncode: int, stderr: str):
        super().__init__(message)
        self.returncode = returncode
        self.stderr = stderr


def find_repo_root(start: Optional[PathLike] = None) -> Path:
    """Locate the bowshock checkout: $BOWSHOCK_REPO, else walk up from ``start`` (or this file) to package.json."""
    env = os.environ.get("BOWSHOCK_REPO")
    if env:
        p = Path(env).expanduser().resolve()
        if (p / "package.json").is_file():
            return p
        raise FileNotFoundError(f"BOWSHOCK_REPO={env} has no package.json")
    here = Path(start).resolve() if start else Path(__file__).resolve().parent
    for d in [here, *here.parents]:
        pj = d / "package.json"
        if pj.is_file():
            try:
                if json.loads(pj.read_text()).get("name") == "bowshock":
                    return d
            except (OSError, ValueError):
                pass
    raise FileNotFoundError("bowshock repo not found; set BOWSHOCK_REPO to the checkout containing package.json")


def run_aerodb(
    config: Union[PathLike, Mapping],
    out_dir: PathLike,
    *,
    repo: Optional[PathLike] = None,
    mode: Optional[str] = None,
    reproducible: bool = False,
    quiet: bool = False,
    timeout: Optional[float] = None,
    npm: Optional[str] = None,
    stream_progress: bool = True,
    extra_args: Sequence[str] = (),
) -> Path:
    """Run the headless CLI (any mode: aerodb, eval, mdo, sweep, sensitivity, fly) and return the output directory.

    ``config`` is a path to a JSON config or a dict (written to a temp file). Progress (stderr) is echoed to
    ``sys.stderr`` when ``stream_progress``; on failure an :class:`AeroDbError` carries the CLI's stderr.
    Ctrl-C forwards SIGINT to the CLI so it cancels cleanly (exit 130, nothing written).
    """
    root = Path(repo).resolve() if repo else find_repo_root()
    npm_bin = npm or shutil.which("npm")
    if not npm_bin:
        raise FileNotFoundError("npm not found on PATH (Node >= 20 is required to run the aero-database CLI)")
    tmp = None
    if isinstance(config, Mapping):
        tmp = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
        json.dump(config, tmp)
        tmp.close()
        cfg_path = Path(tmp.name)
    else:
        cfg_path = Path(config).resolve()
    out = Path(out_dir).resolve()
    args = [npm_bin, "run", "-s", "aerodb", "--", "--config", str(cfg_path), "--out", str(out)]
    if mode:
        args += ["--mode", mode]
    if reproducible:
        args.append("--reproducible")
    if quiet:
        args.append("--quiet")
    args += list(extra_args)
    proc = subprocess.Popen(args, cwd=root, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    err_lines = []
    try:
        assert proc.stderr is not None
        for line in proc.stderr:
            err_lines.append(line)
            if stream_progress:
                sys.stderr.write(line)
        proc.wait(timeout=timeout)
    except KeyboardInterrupt:
        proc.send_signal(signal.SIGINT)
        proc.wait()
        raise
    except subprocess.TimeoutExpired:
        proc.send_signal(signal.SIGINT)
        proc.wait()
        raise
    finally:
        if tmp is not None:
            os.unlink(tmp.name)
    stderr = "".join(err_lines)
    if proc.returncode != 0:
        raise AeroDbError(f"aerodb CLI failed with exit code {proc.returncode}:\n{stderr}", proc.returncode, stderr)
    return out


def main(argv: Optional[Sequence[str]] = None) -> int:
    """``bowshock-aerodb --config cfg.json --out dir [...]``: forwards arguments to the npm CLI."""
    argv = list(sys.argv[1:] if argv is None else argv)
    try:
        root = find_repo_root(os.getcwd())
    except FileNotFoundError:
        root = find_repo_root()
    npm_bin = shutil.which("npm")
    if not npm_bin:
        sys.stderr.write("bowshock-aerodb: npm not found on PATH\n")
        return 1
    # resolve relative paths against the caller's cwd (npm runs in the repo root)
    fixed = []
    it = iter(argv)
    for a in it:
        fixed.append(a)
        if a in ("--config", "--out"):
            v = next(it, None)
            if v is not None:
                fixed.append(str(Path(v).resolve()))
    try:
        return subprocess.call([npm_bin, "run", "-s", "aerodb", "--", *fixed], cwd=root)
    except KeyboardInterrupt:
        return EXIT_INTERRUPTED


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
