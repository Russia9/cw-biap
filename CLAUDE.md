# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this project is

Coursework for BIAP (ballistic/thrust design of a solid-fuel multi-stage rocket). Computations produce Typst-formatted math equations and table rows for a report — output is printed to stdout and pasted into the document.

## Running scripts

```bash
uv run python main.py        # thrust/specific-impulse calculations → Typst math blocks
uv run python preliminary.py # burn-rate and l_z/alpha_dv preliminary tables
```

## Architecture

Three layers:

1. **`assets/*.csv`** — raw data tables digitized from textbook charts:
   - `fuels.csv` — fuel properties (ρ, R, k, T, P_ud, burn-rate law, Al%)
   - `chart-4-26-alpha.csv`, `chart-4-27-l.csv` — digitized nomogram curves for bilinear interpolation
   - `chart-3-5-*.csv`, `chart-3-6-*.csv`, `table-k-k0.csv` — additional reference tables

2. **`utils.py`** — pure physics functions + CSV-backed interpolation. All functions are stateless; they accept SI/practical units and return floats. Chart lookups use `np.interp` with bilinear interpolation across curves.

3. **`main.py` / `preliminary.py`** — calculation scripts. Each defines a `STAGES` list / constants at the top and a `main()` that prints Typst snippets (`#math.equation(...)` blocks or CSV-like table rows).

## Output format

Scripts emit Typst source, not plain text. Inline strings use Typst math syntax with Cyrillic labels (e.g. `"уд"`, `"ст"`). The `eq()` helper in `main.py` wraps a body in a `#math.equation(numbering: none, block: true, $ … $)` call.

## CSV chart format

Multi-curve charts store each curve as a pair of columns (X, Y). Row 0 holds curve labels; row 1 holds `X, Y` headers. Use `pd.read_csv(path, header=1)` for data and a separate `pd.read_csv(path, header=None, nrows=1)` pass to read labels — see `alpha_dv()` in `utils.py` for the pattern.

## Python environment

Use `uv` (see global CLAUDE.md). Python 3.11, dependencies: `pandas`, `numpy` (via pandas), `pandas-stubs`.
