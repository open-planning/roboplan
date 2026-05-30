#!/usr/bin/env python3
"""Post-process a nanobind-generated .pyi stub for a dependent roboplan package.

nanobind's stub generator renders types that live in a *different* extension
module using their C++ qualified name (e.g. ``roboplan::Scene``) rather than the
importable Python name (``roboplan.core.Scene``). Type checkers cannot resolve
the C++ form. Every cross-module roboplan type referenced by a dependent package
lives in ``roboplan-core``, so we rewrite ``roboplan::X`` -> ``roboplan.core.X``
and make sure ``roboplan.core`` is imported so the forward references resolve.

Usage: fix_binding_stub.py <path-to-.pyi>
"""

import re
import sys
from pathlib import Path

# Matches the C++ scope form: roboplan::SomeType  (also inside quoted forward refs)
_CXX_TYPE = re.compile(r"roboplan::([A-Za-z_][A-Za-z0-9_]*)")


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <stub.pyi>", file=sys.stderr)
        return 2

    path = Path(sys.argv[1])
    text = path.read_text()

    if "roboplan::" not in text:
        return 0  # Nothing to fix; leave the file untouched.

    fixed = _CXX_TYPE.sub(r"roboplan.core.\1", text)

    # Ensure roboplan.core is importable so the rewritten references resolve.
    if "import roboplan.core" not in fixed:
        fixed = "import roboplan.core\n" + fixed

    if fixed != text:
        path.write_text(fixed)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
