# Repository Guidelines

## Project Structure & Module Organization

This repository contains BIAP coursework for solid-fuel multistage rocket calculations and external-aero CFD. `main.py` is the primary calculation script and emits Typst math/table snippets. `preliminary.py` emits preliminary design tables. Keep reusable physics, interpolation, and CSV loading logic in `utils.py`; keep Typst formatting helpers in `typst.py`.

Raw textbook data lives in `assets/*.csv`. Geometry is defined in `rocket.scad`; the `Makefile` generates STL variants such as `rocket.stl` and `head.stl`. OpenFOAM tooling is under `openfoam/`: `gen_case.py` creates cases from `templates/`, while `hisa_example/` and `mesh_example/` are upstream references.

## Build, Test, and Development Commands

- `uv run python main.py` prints the main Typst calculation output.
- `uv run python preliminary.py` prints preliminary burn-rate and geometry tables.
- `make stls` builds all configured STL meshes from `rocket.scad`; use `make head.stl` for one part.
- `make png` renders a quick OpenSCAD preview, and `make clean` removes generated meshes/previews.
- `uv run python openfoam/gen_case.py --part all --regime supersonic --Ma 4 --alpha 0` generates an OpenFOAM case. Source the OpenFOAM v2512/HiSA environment before running `./Allrun.pre` or `./Allrun` inside a generated case.

## Coding Style & Naming Conventions

Use Python 3.11 with 4-space indentation and clear type hints for shared helpers. Keep `utils.py` stateless and computation-focused. In `main.py`, preserve the existing split: `calc_*` functions return data and `emit_*` functions print Typst. Use uppercase names for module constants, `snake_case` for functions and variables, and preserve Cyrillic labels in emitted Typst where they are part of the report notation.

## Testing Guidelines

No formal test suite is currently configured. For changes to calculations, verify by running `uv run python main.py` and `uv run python preliminary.py`, then inspect changed numeric output. If adding tests, place them under `tests/` with names like `test_utils.py`, prefer deterministic CSV-backed interpolation checks, and document numeric tolerances explicitly.

## Commit & Pull Request Guidelines

Recent history uses very terse lowercase messages; improve on that with short imperative summaries, for example `update thrust table output`. PRs should describe the calculation or CFD workflow affected, list verification commands run, and note any generated artifacts intentionally included. Include screenshots only for visual geometry or rendered-output changes.

## Agent-Specific Instructions

When using shell commands through Codex in this workspace, prefix commands with `rtk` as required by the local agent configuration. Do not overwrite generated OpenFOAM cases or mesh outputs unless the task explicitly calls for regeneration.
