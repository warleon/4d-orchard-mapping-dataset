#!/usr/bin/env bash
# Regenerates pyrightconfig.json with the ROS PYTHONPATH baked in as
# extraPaths, so LSPs (pyright/basedpyright) can resolve rospy et al.
# without hardcoding nix store paths, which change on every rebuild.
# Run automatically by .envrc whenever the nix devshell (re)loads.
set -euo pipefail

pkg_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

[ -n "${PYTHONPATH:-}" ] || exit 0

python3 - "$pkg_dir/pyrightconfig.json" <<'EOF'
import json
import os
import sys

out_path = sys.argv[1]
ros_paths = [p for p in os.environ.get("PYTHONPATH", "").split(":") if p]

config = {
    "venvPath": ".",
    "venv": ".venv",
    "extraPaths": ["src", *ros_paths],
}

with open(out_path, "w") as f:
    json.dump(config, f, indent=2)
    f.write("\n")
EOF
