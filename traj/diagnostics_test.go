package traj

import (
	"math"
	"testing"
)

// The golden tests only ever see q = 13 059 Pa at the stage-1 separation and
// q = 0 at the stage-2 one, so they cannot exercise the QSepMin boundary, a
// *qualifying* stage-2 separation, or AlphaSepMax picking the larger of two.
// These do.

// sepRows builds the minimal row set diagnose() needs: one row per stage at its
// burnout time, since the separation latch keeps each stage's last active row.
func sepRows(stages int, alphaDeg, q []float64) ([]Row, []float64) {
	rows := make([]Row, 0, stages+1)
	tk := make([]float64, stages)
	for i := range stages {
		tk[i] = float64(i+1) * 10
		rows = append(rows, Row{
			T: tk[i], Stage: i + 1, Alpha: alphaDeg[i] / r2d, Q: q[i],
			H: 30000, V: 1000, Mach: 5,
		})
	}
	// A passive row so burnout/impact reduction has something to land on.
	rows = append(rows, Row{
		T: tk[stages-1] + 1, Stage: stages + 1, H: 30000, V: 1000,
	})
	return rows, tk
}

func alphaSepMax(t *testing.T, stages int, alphaDeg, q []float64) float64 {
	t.Helper()
	r := Rocket{Stages: make([]Stage, stages)}
	rows, tk := sepRows(stages, alphaDeg, q)
	return diagnose(r, rows, tk).AlphaSepMax()
}

func TestAlphaSepMaxQGate(t *testing.T) {
	tests := []struct {
		name     string
		stages   int
		alphaDeg []float64
		q        []float64
		want     float64
	}{{
		name: "inside the atmosphere counts",
		// A two-stage config has exactly one separation: stage 1's burnout.
		stages: 2, alphaDeg: []float64{-2.0, 0}, q: []float64{1500, 0}, want: 2.0,
	}, {
		name:   "below the gate is exempt",
		stages: 2, alphaDeg: []float64{-9.0, 0}, q: []float64{500, 0}, want: 0,
	}, {
		name: "exactly at the gate is exempt",
		// Pins the strict > in AlphaSepMax: QSepMin is the first *excluded*
		// value, so a config sitting on it does not silently acquire a limit.
		stages: 2, alphaDeg: []float64{-9.0, 0}, q: []float64{QSepMin, 0}, want: 0,
	}, {
		name: "takes the max over qualifying separations, not the first",
		// Three stages give two separations; the second is the worse one.
		stages: 3, alphaDeg: []float64{-1.0, -3.0, 0}, q: []float64{1500, 2000, 0},
		want: 3.0,
	}, {
		name:   "a qualifying second separation is caught alone",
		stages: 3, alphaDeg: []float64{-1.0, -4.0, 0}, q: []float64{500, 2000, 0},
		want: 4.0,
	}, {
		name: "final-stage burnout is not a separation",
		// Payload release, even deep in airflow, must not register: it is not a
		// staging event and α is identically zero on the passive side.
		stages: 2, alphaDeg: []float64{-0.5, -9.0}, q: []float64{1500, 5000},
		want: 0.5,
	}}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			got := alphaSepMax(t, tc.stages, tc.alphaDeg, tc.q)
			if math.Abs(got-tc.want) > 1e-9 {
				t.Errorf("AlphaSepMax() = %v, want %v", got, tc.want)
			}
		})
	}
}

func TestCrossUpMarginSignsRelativeToStageTwo(t *testing.T) {
	// The margin is what the optimizer actually follows, so its sign convention
	// is load-bearing: positive inside the stage-2 burn, negative outside.
	const lo, hi = 10.0, 20.0 // stage-2 window for a 2-stage fixture
	tests := []struct {
		name    string
		crossAt float64 // altitude crosses Hatm at this time; 0 = never
		want    float64
	}{
		{"mid-window is most positive", 15.0, 5.0},
		{"just inside the near edge", 11.0, 1.0},
		{"just inside the far edge", 19.5, 0.5},
		{"before stage 2 is negative", 8.0, -2.0},
		{"after stage 2 is negative", 21.0, -1.0},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			r := Rocket{Stages: make([]Stage, 2)}
			tk := []float64{lo, hi}
			// Rows climbing through Hatm at tc.crossAt, one per 0.5 s.
			var rows []Row
			for tt := 0.5; tt <= hi+5; tt += 0.5 {
				stage := 1
				switch {
				case tt > hi:
					stage = 3
				case tt > lo:
					stage = 2
				}
				h := Hatm - 1000
				if tt >= tc.crossAt {
					h = Hatm + 1000
				}
				rows = append(rows, Row{T: tt, Stage: stage, H: h, V: 1000})
			}
			got := diagnose(r, rows, tk).CrossUpMargin
			if math.Abs(got-tc.want) > 1e-9 {
				t.Errorf("CrossUpMargin = %v, want %v", got, tc.want)
			}
		})
	}
}
