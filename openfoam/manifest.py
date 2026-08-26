"""Read-side access to the status manifest `sweep.py` writes.

`convergence.py` and `plot_coeffs.py` are both disk-driven: they pick up
whatever a case directory happens to contain. That is convenient, but it means a
case `sweep.py` gave up on still contributes a row to `averages.csv`, averaged
over the last iterations of a run that never converged.

That is not hypothetical. `stage2up/supersonic/Ma12_a10` timed out after 6 h,
is recorded `failed` in the manifest, and its row sits in `averages.csv` with
Cd_std/|Cd| = 0.385 -- the worst in the table by a factor of ~5.

Only cases the manifest *explicitly* records as not-done are rejected. A case
absent from the manifest is allowed through, so hand-built and ad-hoc cases keep
working; a missing manifest disables the check entirely.
"""

from __future__ import annotations

import json
from pathlib import Path

HERE = Path(__file__).resolve().parent  # openfoam/
DEFAULT_STATE = HERE / "sweep_state.json"

OK_STATUS = "done"


def load_statuses(path: Path | None = None) -> dict[str, str]:
    """Map sweep case id ("<part>/<regime>/<name>") -> status.

    Returns {} when the manifest is absent or unreadable, which disables
    gating rather than blocking the caller.
    """
    p = DEFAULT_STATE if path is None else path
    try:
        raw = json.loads(p.read_text())
    except (OSError, json.JSONDecodeError):
        return {}
    if not isinstance(raw, dict):
        return {}
    return {
        cid: rec.get("status", "")
        for cid, rec in raw.items()
        if isinstance(rec, dict)
    }


def cid(part: str, regime: str, name: str) -> str:
    """Case id, matching sweep.Case.cid."""
    return f"{part}/{regime}/{name}"


def reject_reason(statuses: dict[str, str], part: str, regime: str, name: str) -> str | None:
    """Why this case must not be averaged, or None if it may be used."""
    if not statuses:
        return None
    status = statuses.get(cid(part, regime, name))
    if status is None or status == OK_STATUS:
        return None
    return f"sweep status {status!r} (not {OK_STATUS!r})"
