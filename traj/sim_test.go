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
	if len(rows) != 31974 {
		t.Errorf("rows = %d, pinned 31974", len(rows))
	}
	smokeCheckRows(t, r, rows)
	pinDiagnostics(t, d, map[string]string{
		"MaxQ":            "91102.78798159634",
		"MaxQt":           "36.00000000000024",
		"MaxAlphaSub":     "1.4876043360514415",
		"MaxAlphaSup":     "9.966644515828213",
		"MaxPitchRate":    "2.8407493477549353",
		"MaxPitchRateNum": "2.831091635874278",
		"PitchRateSep1":   "0.4417923556337372",
		"PitchRateSep2":   "0.2916722080013727",
		"CrossUpTime":     "91.19999999999858",
		"CrossUpH":        "94070.68363761343",
		"CrossUpStage":    "2",
		"CrossDownTime":   "3177.4999999982097",
		"BurnoutT":        "142.199999999998",
		"BurnoutV":        "7328.511975119505",
		"BurnoutH":        "269111.86909314804",
		"BurnoutTheta":    "36.96617585443949",
		"ApogeeT":         "1641.1999999996067",
		"ApogeeH":         "3.461435018140195e+06",
		"ImpactT":         "3196.943945310692",
		"ImpactRange":     "1.0233773952114392e+07",
		"GroundHitStage":  "0",
	})
}

// TestSimulateGoldenWithAero pins the reported design point (12 379 km) against
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
	if len(rows) != 24306 {
		t.Errorf("rows = %d, pinned 24306", len(rows))
	}
	smokeCheckRows(t, r, rows)
	pinDiagnostics(t, d, map[string]string{
		"MaxQ":            "76573.91392241052",
		"MaxQt":           "37.600000000000264",
		"MaxAlphaSub":     "1.4876043360514415",
		"MaxAlphaSup":     "9.966644515828213",
		"MaxPitchRate":    "2.921884677948354",
		"MaxPitchRateNum": "2.9216174709593243",
		"PitchRateSep1":   "0.6052707251214638",
		"PitchRateSep2":   "0.32161141434121393",
		"CrossUpTime":     "106.7999999999977",
		"CrossUpH":        "94064.95785348304",
		"CrossUpStage":    "2",
		"CrossDownTime":   "2375.599999998939",
		"BurnoutT":        "142.199999999998",
		"BurnoutV":        "7336.767718819567",
		"BurnoutH":        "166965.0804006718",
		"BurnoutTheta":    "16.472512464552604",
		"ApogeeT":         "1243.899999999968",
		"ApogeeH":         "1.51329392734827e+06",
		"ImpactT":         "2430.3562499988893",
		"ImpactRange":     "1.2379329780130113e+07",
		"GroundHitStage":  "0",
	})
}
