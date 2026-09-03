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
	if len(rows) != 27294 {
		t.Errorf("rows = %d, pinned 27294", len(rows))
	}
	smokeCheckRows(t, r, rows)
	pinDiagnostics(t, d, map[string]string{
		"MaxQ":            "91362.015573128",
		"MaxQt":           "36.10000000000024",
		"MaxAlphaSub":     "1.8320247759743666",
		"MaxAlphaSup":     "17.074154866866685",
		"MaxPitchRate":    "2.9968770423850475",
		"MaxPitchRateNum": "2.996796867768471",
		"PitchRateSep1":   "0.010056346494699213",
		"PitchRateSep2":   "0.31026161489986714",
		// AlphaSep1/AlphaSep2 are byte-identical to the with-aero map below, and
		// must stay that way: an α-framed arc reaches its terminal value exactly
		// at its end time, so the α at a separation is a program constant and
		// cannot depend on aerodynamics. Divergence means the latch is reading
		// the wrong row or the steering frame broke.
		"AlphaSep1":      "7.782461124020066",
		"AlphaSep2":      "9.900948785339244",
		"QSep1":          "6919.869059071404",
		"QSep2":          "0",
		"CrossUpMargin":  "8.400000000001839",
		"CrossUpTime":    "98.69999999999816",
		"CrossUpH":       "94084.66681036912",
		"CrossUpStage":   "2",
		"CrossDownTime":  "2696.599999998647",
		"BurnoutT":       "142.199999999998",
		"BurnoutV":       "7423.289470733936",
		"BurnoutH":       "194504.45287150703",
		"BurnoutTheta":   "19.483864929596443",
		"ApogeeT":        "1401.6999999998245",
		"ApogeeH":        "1.9683163729248196e+06",
		"ImpactT":        "2728.848437498618",
		"ImpactRange":    "1.3009844509082843e+07",
		"GroundHitStage": "0",
	})
}

// TestSimulateGoldenWithAero pins the reported design point (12 749 km) against
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
	if len(rows) != 25639 {
		t.Errorf("rows = %d, pinned 25639", len(rows))
	}
	smokeCheckRows(t, r, rows)
	pinDiagnostics(t, d, map[string]string{
		"MaxQ":            "78813.49868392231",
		"MaxQt":           "38.10000000000027",
		"MaxAlphaSub":     "1.4985310857252372",
		"MaxAlphaSup":     "9.988706207738012",
		"MaxPitchRate":    "2.9968770423850475",
		"MaxPitchRateNum": "2.996796867768471",
		"PitchRateSep1":   "0.010056346494699213",
		"PitchRateSep2":   "0.31026161489986714",
		"AlphaSep1":       "1.4973225270608463",
		"AlphaSep2":       "6.782996932499951",
		"QSep1":           "14805.139880908311",
		"QSep2":           "0",
		"CrossUpMargin":   "0.500000000002288",
		"CrossUpTime":     "106.5999999999977",
		"CrossUpH":        "94000.11660789978",
		"CrossUpStage":    "2",
		"CrossDownTime":   "2512.1999999988147",
		"BurnoutT":        "142.199999999998",
		"BurnoutV":        "7387.581474903258",
		"BurnoutH":        "169785.97343390528",
		"BurnoutTheta":    "17.506654020629938",
		"ApogeeT":         "1312.5999999999055",
		"ApogeeH":         "1.6841885549759418e+06",
		"ImpactT":         "2563.540624998768",
		"ImpactRange":     "1.2749090864529958e+07",
		"GroundHitStage":  "0",
	})
}
