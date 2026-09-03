package traj

import (
	"bufio"
	"encoding/json"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"testing"
)

// TestMetricsJSONContract pins the exact key set of the -metrics line.
// optimize.py reads these keys by name; renaming or dropping one breaks the
// optimizer silently, so any change here must be mirrored there.
func TestMetricsJSONContract(t *testing.T) {
	line := MetricsJSON(Diagnostics{}, Limits{})
	var m map[string]float64
	if err := json.Unmarshal([]byte(line), &m); err != nil {
		t.Fatalf("not a JSON object: %v\n%s", err, line)
	}
	want := []string{
		"apogee_h_km", "apogee_t_s",
		"burnout_h_km", "burnout_theta_deg", "burnout_v",
		"cross_up_margin_s", "cross_up_stage",
		"ground_hit_stage", "impact_range_km", "impact_t_s",
		"lim_eps1", "lim_eps2", "lim_eps_sep",
		"lim_h_max_km", "lim_qmax", "lim_theta_dot",
		"max_alpha_sep_deg", "max_alpha_sub_deg", "max_alpha_sup_deg",
		"max_pitch_rate_dps", "max_pitch_rate_num_dps", "max_q_pa",
	}
	got := make([]string, 0, len(m))
	for k := range m {
		got = append(got, k)
	}
	sort.Strings(got)
	if strings.Join(got, ",") != strings.Join(want, ",") {
		t.Errorf("metrics keys changed:\n got %v\nwant %v", got, want)
	}
	if strings.Contains(line, "\n") {
		t.Error("metrics output must be a single line")
	}
}

func TestMetricsJSONValues(t *testing.T) {
	d := Diagnostics{ImpactRange: 12427359.3, ApogeeH: 1897714.5, GroundHitStage: 2}
	lim := Limits{Eps1: 1.5, HMax: 1800000}
	var m map[string]float64
	if err := json.Unmarshal([]byte(MetricsJSON(d, lim)), &m); err != nil {
		t.Fatal(err)
	}
	if m["impact_range_km"] != 12427.3593 {
		t.Errorf("impact_range_km = %v", m["impact_range_km"])
	}
	if m["lim_h_max_km"] != 1800 {
		t.Errorf("lim_h_max_km = %v", m["lim_h_max_km"])
	}
	if m["ground_hit_stage"] != 2 {
		t.Errorf("ground_hit_stage = %v", m["ground_hit_stage"])
	}
	if m["lim_eps1"] != 1.5 {
		t.Errorf("lim_eps1 = %v", m["lim_eps1"])
	}
}

const csvHeader = "t;m;Vx;Vy;x;y;H;V;vartheta;theta;alpha;Mach;q;X;Y;Mz;omega;stage"

func writeAndRead(t *testing.T, rows []Row, decimate int) []string {
	t.Helper()
	path := filepath.Join(t.TempDir(), "sub", "traj.csv") // exercises MkdirAll
	if err := WriteCSV(rows, path, decimate); err != nil {
		t.Fatal(err)
	}
	f, err := os.Open(path)
	if err != nil {
		t.Fatal(err)
	}
	defer f.Close()
	var lines []string
	sc := bufio.NewScanner(f)
	for sc.Scan() {
		lines = append(lines, sc.Text())
	}
	if err := sc.Err(); err != nil {
		t.Fatal(err)
	}
	return lines
}

func TestWriteCSV(t *testing.T) {
	rows := make([]Row, 5)
	for i := range rows {
		rows[i] = Row{T: float64(i), Stage: 1}
	}

	lines := writeAndRead(t, rows, 1)
	if lines[0] != csvHeader {
		t.Errorf("header = %q, want %q", lines[0], csvHeader)
	}
	if len(lines) != 6 {
		t.Errorf("undecimated: %d lines, want header + 5 rows", len(lines))
	}
	if got := strings.Count(lines[1], ";"); got != 17 {
		t.Errorf("row has %d separators, want 17 (18 columns)", got)
	}

	// decimate=2 keeps rows 0, 2, 4; the last row is always written.
	lines = writeAndRead(t, rows, 2)
	if len(lines) != 4 {
		t.Fatalf("decimated: %d lines, want header + 3 rows", len(lines))
	}
	for i, wantT := range []string{"0.0000", "2.0000", "4.0000"} {
		if !strings.HasPrefix(lines[i+1], wantT+";") {
			t.Errorf("decimated row %d = %q, want t=%s", i, lines[i+1], wantT)
		}
	}

	// decimate=3 would drop row 4 by stride; it must still appear as the final row.
	lines = writeAndRead(t, rows, 3)
	if last := lines[len(lines)-1]; !strings.HasPrefix(last, "4.0000;") {
		t.Errorf("last row = %q, want the final row regardless of stride", last)
	}

	// decimate<=0 behaves like 1.
	if got := len(writeAndRead(t, rows, 0)); got != 6 {
		t.Errorf("decimate=0: %d lines, want 6", got)
	}
}
