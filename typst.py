"""Typst presentation helpers: equation wrappers and parameter-table emission.

Kept separate from utils.py (pure physics) so scripts share one rendering layer.
"""

# Cell contents for a parameter that does not apply to a given stage.
DASH = "$-$"


def emit(body):
    """Print a Typst math body as an unnumbered block equation."""
    print(f"#math.equation(numbering: none, block: true, $ {body} $)")


def fmt(x):
    """Format a number: integer if whole, else up to 6 significant digits."""
    return str(int(x)) if x == int(x) else f"{x:g}"


def section(title, formulas):
    """Open an output section: a comment separator plus the marker line naming
    the formula range it covers, e.g.::

        // ===== Геометрия зарядов и сопел =====

        Проведем расчеты для всех ступеней по формулам $(3.28) - (3.43)$:

    Only substituted calculations and tables follow — the symbolic formulas
    themselves live in the report document, not in this output.
    """
    print(f"// ===== {title} =====")
    print()
    print(f"Проведем расчеты для всех ступеней по формулам ${formulas}$:")
    print()


_STAGE_NAMES = ["Первая", "Вторая", "Третья"]


def stage_header(i):
    """Emit a bold stage heading (1-indexed)."""
    print(f"*{_STAGE_NAMES[i - 1]} ступень:*")


def param_row(label, values, spec=""):
    """Build one Typst parameter-table row: a label cell plus value cells.

    With `spec`, each value is formatted with it and wrapped in `$…$`; without
    it, `values` are taken as already-formatted cell strings (e.g. ``DASH``).
    """
    cells = [f"${v:{spec}}$" for v in values] if spec else list(values)
    return f"    [{label}], {', '.join(cells)},"


def param_rows(entries, items):
    """Build rows for `(label, spec, attr)` triples, reading `attr` off each
    per-stage object in `items`."""
    return [
        param_row(label, [getattr(it, attr) for it in items], spec)
        for label, spec, attr in entries
    ]


def param_table(rows, headers=("I ступень", "II ступень", "III ступень")):
    """Print a Typst ``table(...)`` with a [Параметр] column plus stage columns."""
    print("  table(")
    print(f"    columns: {len(headers) + 1},")
    header_cells = ", ".join(f"[{h}]" for h in headers)
    print(f"    table.header([Параметр], {header_cells}),")
    for row in rows:
        print(row)
    print("  ),")
