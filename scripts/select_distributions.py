#!/usr/bin/env python3
"""Move the wheels and sdists belonging to one project into their own directory.

Filenames are parsed with ``packaging`` so project names compare in canonical form, e.g.,
``roboplan-core`` == ``roboplan_core``.
"""

import argparse
import shutil
import sys
from pathlib import Path

from packaging.utils import (
    canonicalize_name,
    parse_sdist_filename,
    parse_wheel_filename,
)


def project_name(filename: str) -> str:
    """Return the canonical project name of a wheel or sdist filename."""
    if filename.endswith(".whl"):
        return parse_wheel_filename(filename)[0]
    if filename.endswith(".tar.gz"):
        return parse_sdist_filename(filename)[0]
    raise ValueError(f"Not a wheel or sdist: {filename}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("package", help="Project name, e.g., roboplan-core")
    parser.add_argument(
        "--src", type=Path, default=Path("dist"), help="Directory to search"
    )
    parser.add_argument(
        "--dst", type=Path, default=Path("package"), help="Directory to move into"
    )
    args = parser.parse_args()

    wanted = canonicalize_name(args.package)
    args.dst.mkdir(parents=True, exist_ok=True)

    selected = []
    for path in sorted(args.src.iterdir()):
        if path.is_file() and project_name(path.name) == wanted:
            shutil.move(path, args.dst / path.name)
            selected.append(path.name)

    for name in selected:
        print(name)
    if not selected:
        print(f"ERROR: No distributions found for {args.package} in {args.src}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
