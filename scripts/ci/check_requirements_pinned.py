#!/usr/bin/env python3
"""Require every direct dependency in a requirements file to use ==."""

from __future__ import annotations

import argparse
import re
import sys
import tempfile
from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
from pathlib import Path


PINNED_REQUIREMENT = re.compile(
    r"[A-Za-z0-9][A-Za-z0-9._-]*(?:\[[A-Za-z0-9._,-]+\])?"
    r"==[A-Za-z0-9][A-Za-z0-9.!+_-]*"
)


def check_requirements(path: Path) -> bool:
    valid = True
    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw_line.split("#", 1)[0].strip()
        if not line or line.startswith("--extra-index-url "):
            continue

        requirement = line.split(";", 1)[0].strip()
        if PINNED_REQUIREMENT.fullmatch(requirement):
            print(f"[PASS] {path}:{line_number}: {requirement}")
        else:
            print(
                f"[FAIL] {path}:{line_number}: {requirement!r} must use an exact == pin",
                file=sys.stderr,
            )
            valid = False
    return valid


def run_self_test() -> bool:
    with tempfile.TemporaryDirectory() as temporary_directory:
        requirements_path = Path(temporary_directory) / "requirements.txt"
        requirements_path.write_text("numpy>=2,<3\n", encoding="utf-8")
        expected_error = StringIO()
        with redirect_stdout(StringIO()), redirect_stderr(expected_error):
            unpinned_accepted = check_requirements(requirements_path)
        if unpinned_accepted:
            print("[FAIL] Self-test accepted an unpinned requirement", file=sys.stderr)
            return False
        print(f"[EXPECTED FAIL] {expected_error.getvalue().strip()}")
        print("[PASS] Self-test rejected an unpinned requirement")

        requirements_path.write_text("numpy==2.2.6\n", encoding="utf-8")
        with redirect_stdout(StringIO()), redirect_stderr(StringIO()):
            pinned_accepted = check_requirements(requirements_path)
        if not pinned_accepted:
            print("[FAIL] Self-test rejected a pinned requirement", file=sys.stderr)
            return False
        print("[PASS] Self-test accepted an exact pin: numpy==2.2.6")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("requirements", nargs="?", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test and not run_self_test():
        return 1
    if args.requirements and not check_requirements(args.requirements):
        return 1
    if not args.self_test and not args.requirements:
        parser.error("provide a requirements file or --self-test")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())