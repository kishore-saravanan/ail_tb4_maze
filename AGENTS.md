# Repository conventions

- Keep Python readable and follow the Ruff configuration in `pyproject.toml`.
- Always leave a blank line immediately before each Python `return` statement,
  including when it is the first statement in a function or conditional block.
- Run Ruff formatting first, then `python3 scripts/check_return_spacing.py --fix`.
  Ruff can remove blank lines at the beginning of blocks; the spacing helper
  reapplies this repository's explicit convention.
- Validate with `ruff check .` and `python3 scripts/check_return_spacing.py`.
- Keep the local 2D testbed, runtime files and raw recordings out of Git.
