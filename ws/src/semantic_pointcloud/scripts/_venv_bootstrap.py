"""Re-exec the current process under this package's uv-managed .venv.

catkin runs installed python scripts through a relay stub whose shebang is
whatever system python CMake found, so a node script's own shebang line is
never actually honored. To get at dependencies installed via `uv sync`
(e.g. torch), a node must instead re-exec itself into
`<package_root>/.venv/bin/python3` at runtime. ROS packages (rospy and
friends) stay reachable afterwards because they're found via PYTHONPATH,
which os.execv preserves, not via the venv's site-packages.

Usage, at the very top of a node script, before importing anything from
the venv (torch, etc.):

    from _venv_bootstrap import activate
    activate(__file__)
"""

import os
import sys


def activate(caller_file):
    package_root = os.path.dirname(os.path.dirname(os.path.abspath(caller_file)))
    venv_dir = os.path.join(package_root, ".venv")
    venv_python = os.path.join(venv_dir, "bin", "python3")

    # uv venvs symlink their python binary straight to the base interpreter,
    # so comparing sys.executable can't tell venv and system python apart;
    # sys.prefix is what actually changes once the venv is active.
    if os.path.realpath(sys.prefix) == os.path.realpath(venv_dir):
        return

    if not os.path.exists(venv_python):
        raise RuntimeError(
            f"expected a uv-managed venv at {venv_python!r}; run `uv sync` in "
            f"{package_root!r} first"
        )

    os.execv(venv_python, [venv_python, os.path.abspath(caller_file), *sys.argv[1:]])
