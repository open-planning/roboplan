#!/usr/bin/env python3
"""Set the version of every roboplan Python package, including their pins on each other."""

import argparse
import re
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("version", help="New version, e.g., 0.7.0.dev0")
    args = parser.parse_args()

    pattern = re.compile(r'(^version = "|roboplan-[a-z-]+ ==)[\d.]+', re.MULTILINE)
    for pyproject in Path(__file__).parent.parent.glob("roboplan*/pyproject.toml"):
        text = pattern.sub(rf"\g<1>{args.version}", pyproject.read_text())
        pyproject.write_text(text)


if __name__ == "__main__":
    main()
