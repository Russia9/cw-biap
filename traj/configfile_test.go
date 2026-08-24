package traj

import (
	"math"
	"strings"
	"testing"
)

func TestDefaultConfigParses(t *testing.T) {
	r, lim := DefaultConfig() // panics if the embedded rocket.json is invalid
	if len(r.Stages) != 3 {
		t.Fatalf("stages = %d, want 3", len(r.Stages))
	}
	if r.Payload <= 0 || r.PayloadPart == "" {
		t.Errorf("payload = %v %q, want positive mass and a part key", r.Payload, r.PayloadPart)
	}
	if lim.Eps1 <= 0 || lim.Eps2 <= 0 || lim.ThetaDotMax <= 0 || lim.Qmax <= 0 {
		t.Errorf("limits not populated: %+v", lim)
	}
	if err := r.Pitch.Validate(); err != nil {
		t.Errorf("embedded pitch program invalid: %v", err)
	}
	// The program must span exactly the powered flight.
	tk := r.BurnoutTimes()
	last := r.Pitch.Segments[len(r.Pitch.Segments)-1]
	if math.Abs(last.TEnd-tk[len(tk)-1]) > 1e-9 {
		t.Errorf("last segment ends at %v, burnout at %v", last.TEnd, tk[len(tk)-1])
	}
}

func TestLoadConfigMatchesDefault(t *testing.T) {
	// LoadConfig("rocket.json") and the embedded default are the same file.
	r, lim, err := LoadConfig("rocket.json")
	if err != nil {
		t.Fatal(err)
	}
	dr, dlim := DefaultConfig()
	if lim != dlim {
		t.Errorf("limits differ: %+v vs %+v", lim, dlim)
	}
	if len(r.Stages) != len(dr.Stages) || len(r.Pitch.Segments) != len(dr.Pitch.Segments) {
		t.Fatalf("shape differs: %d/%d stages, %d/%d segments",
			len(r.Stages), len(dr.Stages), len(r.Pitch.Segments), len(dr.Pitch.Segments))
	}
	for i := range r.Stages {
		if r.Stages[i] != dr.Stages[i] {
			t.Errorf("stage %d differs: %+v vs %+v", i+1, r.Stages[i], dr.Stages[i])
		}
	}
	for i := range r.Pitch.Segments {
		a, b := r.Pitch.Segments[i], dr.Pitch.Segments[i]
		if a.TEnd != b.TEnd || a.Val != b.Val || a.Shape != b.Shape || a.Frame != b.Frame || a.K != b.K {
			t.Errorf("segment %d differs: %+v vs %+v", i, a, b)
		}
	}
}

func TestParseConfigErrors(t *testing.T) {
	base := `{"payload_mass": 620, "payload_part": "head", "t_vertical": 5,
		"stages": [%s],
		"limits": {"eps1": 1.5, "eps2": 10, "theta_dot_max": 3, "qmax": 120000}}`
	stage := `{"m0": 1000, "m_fuel": 800, "burn_time": 60, "isp_sl": 220, "isp_vac": 250,
		"part": "all"%s, "pitch": [%s]}`

	cases := []struct{ name, json, wantErr string }{
		{"no stages",
			`{"payload_mass": 620, "stages": [], "limits": {}}`,
			"no stages"},
		{"not json",
			`{`,
			"parse config"},
		{"theta stage needs theta_deg",
			sprintf2(base, stage, ``, `{"alpha_deg": 1.0}`),
			"theta-steered arc needs theta_deg"},
		{"theta stage rejects alpha_deg",
			sprintf2(base, stage, ``, `{"theta_deg": 40, "alpha_deg": 1.0}`),
			"theta-steered arc must not set alpha_deg"},
		{"alpha stage needs alpha_deg",
			sprintf2(base, stage, `, "steering": "alpha"`, `{"theta_deg": 40}`),
			"alpha-steered arc needs alpha_deg"},
		{"alpha stage rejects theta_deg",
			sprintf2(base, stage, `, "steering": "alpha"`, `{"theta_deg": 40, "alpha_deg": 1.0}`),
			"alpha-steered arc must not set theta_deg"},
		{"unknown steering",
			sprintf2(base, stage, `, "steering": "roll"`, `{"theta_deg": 40}`),
			"unknown steering"},
		{"unknown shape",
			sprintf2(base, stage, ``, `{"theta_deg": 40, "shape": "spline"}`),
			"unknown shape"},
		{"non-final arc needs t_end",
			sprintf2(base, stage, ``, `{"theta_deg": 60}, {"theta_deg": 40}`),
			"non-final arc needs t_end"},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			_, _, err := parseConfig([]byte(c.json))
			if err == nil {
				t.Fatalf("want error containing %q, got nil", c.wantErr)
			}
			if !strings.Contains(err.Error(), c.wantErr) {
				t.Errorf("error = %q, want it to contain %q", err, c.wantErr)
			}
		})
	}
}

// sprintf2 fills the stage template into the base template.
func sprintf2(base, stage, steering, pitch string) string {
	st := strings.Replace(stage, "%s", steering, 1)
	st = strings.Replace(st, "%s", pitch, 1)
	return strings.Replace(base, "%s", st, 1)
}

func TestParseConfigDefaults(t *testing.T) {
	// Shape defaults to exp with defaultKExp; explicit k wins; cos gets defaultKCos.
	json := `{"payload_mass": 620, "payload_part": "head", "t_vertical": 5,
		"stages": [{"m0": 1000, "m_fuel": 800, "burn_time": 60, "isp_sl": 220, "isp_vac": 250,
			"part": "all", "pitch": [
				{"theta_deg": 80, "t_end": 20},
				{"theta_deg": 60, "t_end": 40, "shape": "cos"},
				{"theta_deg": 40, "k": 2.5}]}],
		"limits": {"eps1": 1.5, "eps2": 10, "theta_dot_max": 3, "qmax": 120000}}`
	r, _, err := parseConfig([]byte(json))
	if err != nil {
		t.Fatal(err)
	}
	segs := r.Pitch.Segments
	if segs[0].Shape != ShapeExp || segs[0].K != defaultKExp {
		t.Errorf("arc 1: shape/k = %v/%v, want exp default %v", segs[0].Shape, segs[0].K, defaultKExp)
	}
	if segs[1].Shape != ShapeCos || segs[1].K != defaultKCos {
		t.Errorf("arc 2: shape/k = %v/%v, want cos default %v", segs[1].Shape, segs[1].K, defaultKCos)
	}
	if segs[2].K != 2.5 {
		t.Errorf("arc 3: k = %v, want explicit 2.5", segs[2].K)
	}
	// The final arc's TEnd is the cumulative burnout time.
	if segs[2].TEnd != 60 {
		t.Errorf("final arc TEnd = %v, want burn_time 60", segs[2].TEnd)
	}
}
