"""Unit tests for optimize.py's vector layout and objective plumbing.

Run from traj/:  uv run python -m unittest test_optimize -v
"""

import copy
import json
import unittest
from pathlib import Path

from optimize import (
    F_FAIL,
    Config,
    Layout,
    SimError,
    SimRunner,
    arc_k,
    objective,
)

HERE = Path(__file__).parent

# A small two-stage config exercising both steering frames, shape defaults and
# an explicit k.
BASE: Config = {
    "payload_mass": 620.0,
    "payload_part": "head",
    "t_vertical": 5.0,
    "stages": [
        {
            "m0": 1000.0,
            "m_fuel": 800.0,
            "burn_time": 60.0,
            "isp_sl": 220.0,
            "isp_vac": 250.0,
            "part": "all",
            "pitch": [
                {"theta_deg": 80.0, "t_end": 20.0},
                {"theta_deg": 60.0, "shape": "cos", "k": 2.0},
            ],
        },
        {
            "m0": 100.0,
            "m_fuel": 80.0,
            "burn_time": 40.0,
            "isp_sl": 220.0,
            "isp_vac": 250.0,
            "part": "stage2up",
            "steering": "alpha",
            "pitch": [
                {"alpha_deg": -1.0, "t_end": 80.0},
                {"alpha_deg": 0.5},
            ],
        },
    ],
    "limits": {
        "eps1": 1.5,
        "eps2": 10.0,
        "theta_dot_max": 3.0,
        "qmax": 120000.0,
        "h_max": 1800000.0,
    },
}


class LayoutTest(unittest.TestCase):
    def setUp(self) -> None:
        self.layout = Layout.from_config(copy.deepcopy(BASE))

    def test_vector_blocks_agree(self) -> None:
        dim = self.layout.vector_dim()
        self.assertEqual(dim, 2 * 4 + 1 + 2)  # 4 arcs, 2 non-final
        self.assertEqual(len(self.layout.default_x0()), dim)
        lo, hi = self.layout.bounds()
        self.assertEqual(len(lo), dim)
        self.assertEqual(len(hi), dim)
        self.assertEqual(len(self.layout.cma_stds()), dim)
        self.assertTrue(all(a < b for a, b in zip(lo, hi)))

    def test_default_x0_round_trips(self) -> None:
        x0 = self.layout.default_x0()
        cfg = self.layout.config_from_x(x0)
        for sp in self.layout.specs:
            base_arc = self.layout.base_arc(sp)
            got_arc = cfg["stages"][sp.stage]["pitch"][sp.arc]
            key = "alpha_deg" if sp.frame == "alpha" else "theta_deg"
            self.assertEqual(got_arc[key], base_arc[key])
            self.assertEqual(got_arc["k"], arc_k(base_arc))
            if not sp.is_final:
                self.assertEqual(got_arc["t_end"], base_arc["t_end"])
        self.assertEqual(cfg["t_vertical"], BASE["t_vertical"])
        # Physical fields pass through untouched.
        self.assertEqual(cfg["stages"][0]["m0"], 1000.0)
        self.assertEqual(cfg["limits"], BASE["limits"])

    def test_unpack_rejects_wrong_length(self) -> None:
        with self.assertRaises(ValueError):
            self.layout.unpack([1.0, 2.0])

    def test_alpha_entry_bounds(self) -> None:
        base = copy.deepcopy(BASE)
        base["stages"][1]["pitch"][0]["theta0_deg"] = -0.5
        layout = Layout.from_config(base)
        self.assertEqual(len(layout.entries), 1)
        lo, hi = layout.bounds()
        # The entry block is the trailing dimension; an α-framed entry must get
        # the α bounds, not 0..89.
        self.assertLess(lo[-1], 0.0)
        self.assertLessEqual(hi[-1], 10.0)

    def test_steering_flip_names_the_missing_key(self) -> None:
        base = copy.deepcopy(BASE)
        base["stages"][0]["steering"] = "alpha"  # arcs still carry theta_deg
        layout = Layout.from_config(base)
        with self.assertRaises(SystemExit) as ctx:
            layout.default_x0()
        self.assertIn("alpha_deg", str(ctx.exception))
        self.assertIn("stage 1", str(ctx.exception))

    def test_config_from_x_drops_stale_opposite_angle(self) -> None:
        base = copy.deepcopy(BASE)
        # A flipped stage where the user added alpha_deg but left theta_deg.
        base["stages"][0]["steering"] = "alpha"
        for arc in base["stages"][0]["pitch"]:
            arc["alpha_deg"] = -1.0
        layout = Layout.from_config(base)
        cfg = layout.config_from_x(layout.default_x0())
        for arc in cfg["stages"][0]["pitch"]:
            self.assertNotIn("theta_deg", arc)
            self.assertIn("alpha_deg", arc)

    def test_program_penalty(self) -> None:
        x0 = self.layout.default_x0()
        self.assertEqual(self.layout.program_penalty(x0), 0.0)

        # Push the first split past the second arc's end: ordering violation.
        bad = list(x0)
        bad[4 + 1] = 61.0  # first split time (4 angles, then t_в)
        self.assertGreater(self.layout.program_penalty(bad), 0.0)

        # A rising ϑ is NOT penalised: nulling α for an in-atmosphere stage
        # separation requires exactly that, so the old monotonicity term was
        # removed rather than exempted.
        bad = list(x0)
        bad[0], bad[1] = 60.0, 80.0
        self.assertEqual(self.layout.program_penalty(bad), 0.0)

        # Rising α likewise costs nothing.
        bad = list(x0)
        bad[2], bad[3] = -1.0, 0.5
        self.assertEqual(self.layout.program_penalty(bad), 0.0)

    def test_rocket_json_round_trips(self) -> None:
        with open(HERE / "rocket.json") as f:
            base = json.load(f)
        layout = Layout.from_config(base)
        cfg = layout.config_from_x(layout.default_x0())
        # The generated config differs from the base only by explicit "k" on
        # arcs that relied on the default.
        for sp in layout.specs:
            want = dict(layout.base_arc(sp))
            want.setdefault("k", arc_k(want))
            self.assertEqual(cfg["stages"][sp.stage]["pitch"][sp.arc], want)


class StubRunner(SimRunner):
    """A SimRunner whose metrics are canned, for objective() tests."""

    _result: dict[str, float] | Exception

    def __init__(self, result: dict[str, float] | Exception) -> None:
        super().__init__(binary=Path("unused"), cwd=Path("."), aero="")
        object.__setattr__(self, "_result", result)  # the base class is frozen

    def metrics(self, cfg: Config, h: float) -> dict[str, float]:
        if isinstance(self._result, Exception):
            raise self._result
        return self._result


class ObjectiveTest(unittest.TestCase):
    def setUp(self) -> None:
        self.layout = Layout.from_config(copy.deepcopy(BASE))
        self.x0 = self.layout.default_x0()

    def feasible_metrics(self) -> dict[str, float]:
        return {
            "impact_range_km": 12000.0,
            "max_alpha_sub_deg": 1.0,
            "max_alpha_sup_deg": 5.0,
            "max_pitch_rate_dps": 2.0,
            "max_pitch_rate_num_dps": 2.0,
            "max_q_pa": 80000.0,
            "apogee_h_km": 1500.0,
            "max_alpha_sep_deg": 1.0,
            "cross_up_stage": 2.0,
            "cross_up_margin_s": 20.0,
            "lim_eps1": 1.5,
            "lim_eps2": 10.0,
            "lim_theta_dot": 3.0,
            "lim_qmax": 120000.0,
            "lim_h_max_km": 1800.0,
            "lim_eps_sep": 1.5,
        }

    def test_on_target_feasible_is_near_zero(self) -> None:
        runner = StubRunner(self.feasible_metrics())
        self.assertAlmostEqual(
            objective(self.layout, runner, self.x0, 12000.0, 0.1), 0.0
        )

    def test_violation_dominates_range_error(self) -> None:
        m = self.feasible_metrics()
        m["max_q_pa"] = 150000.0
        runner = StubRunner(m)
        f = objective(self.layout, runner, self.x0, 12000.0, 0.1)
        self.assertGreater(f, 1e4)

    def test_separation_alpha_violation_is_penalised(self) -> None:
        m = self.feasible_metrics()
        m["max_alpha_sep_deg"] = 7.65  # the pre-constraint stage-1 separation
        runner = StubRunner(m)
        f = objective(self.layout, runner, self.x0, 12000.0, 0.1)
        self.assertGreater(f, 1e4)

    def test_exempt_separation_costs_nothing(self) -> None:
        # The Go side applies the q gate and reports 0 for a separation outside
        # the atmosphere, so a large α up there must not register as a
        # violation here.
        m = self.feasible_metrics()
        m["max_alpha_sep_deg"] = 0.0
        runner = StubRunner(m)
        self.assertAlmostEqual(
            objective(self.layout, runner, self.x0, 12000.0, 0.1), 0.0
        )

    def test_crossing_outside_stage_two_is_penalised(self) -> None:
        m = self.feasible_metrics()
        m["cross_up_stage"], m["cross_up_margin_s"] = 3.0, -0.9
        runner = StubRunner(m)
        f = objective(self.layout, runner, self.x0, 12000.0, 0.1)
        self.assertGreater(f, 1e4)

    def test_crossing_penalty_grades_with_distance(self) -> None:
        # Penalising cross_up_stage directly would score these identically and
        # leave CMA-ES no direction to move; the margin form must rank the near
        # miss strictly better than the far one.
        near, far = self.feasible_metrics(), self.feasible_metrics()
        near["cross_up_margin_s"], far["cross_up_margin_s"] = -0.5, -5.0
        self.assertLess(
            objective(self.layout, StubRunner(near), self.x0, 12000.0, 0.1),
            objective(self.layout, StubRunner(far), self.x0, 12000.0, 0.1),
        )

    def test_crossing_on_the_staging_instant_is_penalised(self) -> None:
        # Landing exactly on the boundary must still cost something:
        # CROSS_MARGIN_S is what keeps the converged solution off the
        # discontinuity, where a step-size change could flip it.
        m = self.feasible_metrics()
        m["cross_up_margin_s"] = 0.0
        f = objective(self.layout, StubRunner(m), self.x0, 12000.0, 0.1)
        self.assertGreater(f, 0.0)

    def test_sim_error_scores_f_fail(self) -> None:
        runner = StubRunner(SimError("boom", stderr="load config: ..."))
        f = objective(self.layout, runner, self.x0, 12000.0, 0.1)
        self.assertGreaterEqual(f, F_FAIL)

    def test_zero_limits_do_not_divide(self) -> None:
        m = self.feasible_metrics()
        for key in (
            "lim_eps1",
            "lim_eps2",
            "lim_theta_dot",
            "lim_qmax",
            "lim_h_max_km",
            "lim_eps_sep",
        ):
            m[key] = 0.0
        runner = StubRunner(m)
        # Must not raise ZeroDivisionError; zero limits mean no penalty terms.
        self.assertAlmostEqual(
            objective(self.layout, runner, self.x0, 12000.0, 0.1), 0.0
        )


if __name__ == "__main__":
    unittest.main()
