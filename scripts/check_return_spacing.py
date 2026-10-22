#!/usr/bin/env python3
"""Check or insert a blank line before every tracked Python return statement."""

import argparse
import ast
import subprocess
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fix", action="store_true", help="Insert missing blank lines")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    names = (
        subprocess.check_output(
            ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
            cwd=root,
        )
        .decode()
        .split("\0")
    )
    problems = 0
    for name in sorted(set(names)):
        path = root / name
        if not path.is_file():
            continue
        source = path.read_bytes()
        if path.suffix != ".py" and not (
            name.startswith("resource/") and b"python" in source.split(b"\n", 1)[0]
        ):
            continue
        text = source.decode()
        tree = ast.parse(text, filename=name)
        lines = text.splitlines(keepends=True)
        missing = sorted(
            {
                node.lineno - 1
                for node in ast.walk(tree)
                if isinstance(node, ast.Return)
                and node.lineno > 1
                and lines[node.lineno - 2].strip()
            }
        )
        for index in missing:
            if not lines[index].lstrip().startswith("return"):
                raise ValueError(f"Expand inline return first: {name}:{index + 1}")
            if not args.fix:
                print(f"{name}:{index + 1}: blank line required before return")
        problems += len(missing)
        if args.fix and missing:
            for index in reversed(missing):
                lines.insert(index, "\n")
            updated = "".join(lines)
            assert ast.dump(ast.parse(updated)) == ast.dump(tree)
            path.write_text(updated)
    if args.fix:
        print(f"Inserted {problems} blank lines; Python syntax trees unchanged.")

    return 0 if args.fix or not problems else 1


if __name__ == "__main__":
    raise SystemExit(main())
