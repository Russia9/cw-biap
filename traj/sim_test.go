package traj

import (
	"fmt"
	"math"
	"reflect"
	"testing"
)

// pinDiagnostics compares every Diagnostics field against its pinned value
// (shortest-round-trip %v strings, so matches are bit-exact). Reflection makes
// a missing or stale pin a failure too: adding a field forces a new pin.
func pinDiagnostics(t *testing.T, d Diagnostics, want map[string]string) {
	t.Helper()
	v := reflect.ValueOf(d)
	for i := 0; i < v.NumField(); i++ {
		name := v.Type().Field(i).Name
		have := fmt.Sprintf("%v", v.Field(i).Interface())
		pin, ok := want[name]
		if !ok {
			t.Errorf("field %s has no pin (value %s)", name, have)
			continue
		}
		if have != pin {
			t.Errorf("%s = %s, pinned %s", name, have, pin)
		}
		delete(want, name)
	}
	for name := range want {
		t.Errorf("pin %s matches no Diagnostics field", name)
	}
}

// smokeCheckRows asserts the structural invariants of a simulated trajectory:
// monotone time, correct stage tagging and ordering, exact per-stage mass
// bookkeeping (dm/dt is constant, so RK4 integrates it exactly), constant
// payload mass, and impact within the ground-stop tolerance.
func smokeCheckRows(t *testing.T, r Rocket, rows []Row) {
	t.Helper()
	tk := r.BurnoutTimes()
	passiveTag := len(r.Stages) + 1
	prevT := math.Inf(-1)
	prevStage := 0
	for i, row := range rows {
		if row.T <= prevT {
			t.Fatalf("row %d: time not increasing (%.6f after %.6f)", i, row.T, prevT)
		}
		prevT = row.T
		if row.Stage < prevStage {
			t.Fatalf("row %d: stage %d after stage %d", i, row.Stage, prevStage)
		}
		prevStage = row.Stage
		if row.Stage >= 1 && row.Stage <= len(r.Stages) {
			st := r.Stages[row.Stage-1]
			t0 := 0.0
			if row.Stage > 1 {
				t0 = tk[row.Stage-2]
			}
			wantM := st.M0 - st.MassFlow()*(row.T-t0)
			if math.Abs(row.M-wantM) > 1e-6 {
				t.Fatalf("row %d (t=%.2f, stage %d): m=%.9f, want %.9f", i, row.T, row.Stage, row.M, wantM)
			}
		} else if row.Stage == passiveTag {
			if row.M != r.Payload {
				t.Fatalf("row %d: passive mass %.9f, want payload %.9f", i, row.M, r.Payload)
			}
		} else {
			t.Fatalf("row %d: unexpected stage tag %d", i, row.Stage)
		}
	}
	last := rows[len(rows)-1]
	if math.Abs(last.H) >= 1.0 {
		t.Errorf("impact row H = %.4f m, want |H| < 1 (stopGround tolerance)", last.H)
	}
}

// TestSimulateGoldenZeroAero pins the full flight of the embedded config with
// zero aerodynamics — hermetic (no CSV dependency). The pins change only when
// the physics changes, and then deliberately.
func TestSimulateGoldenZeroAero(t *testing.T) {
	r, _ := DefaultConfig()
	rows, d, err := Simulate(r, ZeroAero(), 0.1)
	if err != nil {
		t.Fatal(err)
	}
	if len(rows) != 28283 {
		t.Errorf("rows = %d, pinned 28283", len(rows))
	}
	smokeCheckRows(t, r, rows)
	pinDiagnostics(t, d, map[string]string{
		"MaxQ":            "91211.05957846092",
		"MaxQt":           "35.80000000000024",
		"MaxAlphaSub":     "1.638934600826113",
		"MaxAlphaSup":     "15.99855290313276",
		"MaxPitchRate":    "2.997022033095309",
		"MaxPitchRateNum": "3.027442781117401",
		"PitchRateSep1":   "0",
		"PitchRateSep2":   "1.5472707971050502e-13",
		"CrossUpTime":     "95.19999999999835",
		"CrossUpH":        "94133.99353979249",
		"CrossUpStage":    "2",
		"CrossDownTime":   "2798.599999998554",
		"BurnoutT":        "139.79999999999825",
		"BurnoutV":        "7421.260688094434",
		"BurnoutH":        "206345.31038725842",
		"BurnoutTheta":    "22.183377609775967",
		"ApogeeT":         "1451.1999999997793",
		"ApogeeH":         "2.2169063814210016e+06",
		"ImpactT":         "2827.822265623527",
		"ImpactRange":     "1.2747352136270216e+07",
		"GroundHitStage":  "0",
	})
}

// TestSimulateGoldenWithAero pins the reported design point (12 427 km) against
// the CFD table. Skipped when the table is not checked out.
func TestSimulateGoldenWithAero(t *testing.T) {
	at, err := LoadAero("../openfoam/results/averages.csv")
	if err != nil {
		t.Skipf("CFD table unavailable: %v", err)
	}
	r, _ := DefaultConfig()
	rows, d, err := Simulate(r, at, 0.1)
	if err != nil {
		t.Fatal(err)
	}
	if len(rows) != 26320 {
		t.Errorf("rows = %d, pinned 26320", len(rows))
	}
	smokeCheckRows(t, r, rows)
	pinDiagnostics(t, d, map[string]string{
		"MaxQ":            "76391.14858549953",
		"MaxQt":           "37.000000000000256",
		"MaxAlphaSub":     "1.5013267485818862",
		"MaxAlphaSup":     "10.036204536519318",
		"MaxPitchRate":    "2.997022033095309",
		"MaxPitchRateNum": "3.027442781117401",
		"PitchRateSep1":   "0",
		"PitchRateSep2":   "1.5472707971050502e-13",
		"CrossUpTime":     "102.39999999999795",
		"CrossUpH":        "94031.696174765",
		"CrossUpStage":    "2",
		"CrossDownTime":   "2587.399999998746",
		"BurnoutT":        "139.79999999999825",
		"BurnoutV":        "7363.75307783533",
		"BurnoutH":        "181731.52799278405",
		"BurnoutTheta":    "20.236943523280193",
		"ApogeeT":         "1348.2999999998729",
		"ApogeeH":         "1.8977141774933217e+06",
		"ImpactT":         "2631.593749998706",
		"ImpactRange":     "1.242735885162955e+07",
		"GroundHitStage":  "0",
	})
}
