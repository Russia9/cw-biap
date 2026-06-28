package traj

import (
	"bufio"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"
)

const r2d = 180 / math.Pi

// WriteCSV writes the trajectory as a semicolon-separated file (pandas sep=";").
// Every `decimate`-th row is written (decimate<=1 writes all rows).
func WriteCSV(rows []Row, path string, decimate int) error {
	if decimate < 1 {
		decimate = 1
	}
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		return err
	}
	f, err := os.Create(path)
	if err != nil {
		return err
	}
	defer f.Close()
	w := bufio.NewWriter(f)
	defer w.Flush()

	fmt.Fprintln(w, "t;m;Vx;Vy;x;y;H;V;vartheta;theta;alpha;Mach;q;X;Y;Mz;omega;stage")
	for i, r := range rows {
		if i%decimate != 0 && i != len(rows)-1 {
			continue
		}
		fmt.Fprintf(w, "%.4f;%.3f;%.4f;%.4f;%.3f;%.3f;%.3f;%.4f;%.4f;%.4f;%.4f;%.5f;%.4f;%.3f;%.3f;%.3f;%.6f;%d\n",
			r.T, r.M, r.Vx, r.Vy, r.Xpos, r.Ypos, r.H, r.V,
			r.Pitch*r2d, r.Theta*r2d, r.Alpha*r2d, r.Mach, r.Q,
			r.Drag, r.Lift, r.Mz, r.Omega*r2d, r.Stage)
	}
	return nil
}

// MetricsJSON returns a single-line JSON object with the terminal trajectory
// parameters, the measured §4.4 maxima, and the configured limits. It is the
// machine-readable interface consumed by the CMA-ES optimizer (optimize.py).
func MetricsJSON(d Diagnostics, lim Limits) string {
	m := map[string]float64{
		"impact_range_km":    d.ImpactRange / 1000,
		"impact_t_s":         d.ImpactT,
		"apogee_h_km":        d.ApogeeH / 1000,
		"apogee_t_s":         d.ApogeeT,
		"burnout_v":          d.BurnoutV,
		"burnout_h_km":       d.BurnoutH / 1000,
		"burnout_theta_deg":  d.BurnoutTheta,
		"max_alpha_sub_deg":  d.MaxAlphaSub,
		"max_alpha_sup_deg":  d.MaxAlphaSup,
		"max_pitch_rate_dps": d.MaxPitchRate,
		"max_q_pa":           d.MaxQ,
		"lim_eps1":           lim.Eps1,
		"lim_eps2":           lim.Eps2,
		"lim_theta_dot":      lim.ThetaDotMax,
		"lim_qmax":           lim.Qmax,
	}
	b, _ := json.Marshal(m)
	return string(b)
}

func okFlag(measured, limit float64) string {
	if measured <= limit {
		return "OK"
	}
	return "EXCEEDS"
}

// PrintDiagnostics prints the constructive-ballistic checks (§4.4) and terminal
// trajectory parameters with pass/fail against the configured limits.
func PrintDiagnostics(d Diagnostics, lim Limits, at *AeroTable) {
	fmt.Println("=== Constraint diagnostics (§4.4) ===")
	fmt.Printf("  max |α|, M≤1.1            : %6.2f deg   (limit %.2f)  %s\n", d.MaxAlphaSub, lim.Eps1, okFlag(d.MaxAlphaSub, lim.Eps1))
	fmt.Printf("  max |α|, M>1.1 & H≤94km   : %6.2f deg   (limit %.2f)  %s\n", d.MaxAlphaSup, lim.Eps2, okFlag(d.MaxAlphaSup, lim.Eps2))
	fmt.Printf("  max |ϑ̇| (active)          : %6.2f deg/s (limit %.2f)  %s\n", d.MaxPitchRate, lim.ThetaDotMax, okFlag(d.MaxPitchRate, lim.ThetaDotMax))
	fmt.Printf("  |ϑ̇| at stage-1 sep        : %6.3f deg/s (≈0 for smooth separation)\n", d.PitchRateSep1)
	fmt.Printf("  |ϑ̇| at stage-2 sep        : %6.3f deg/s (≈0 for smooth separation)\n", d.PitchRateSep2)
	fmt.Printf("  max q                     : %8.1f Pa  (limit %.0f)  %s\n", d.MaxQ, lim.Qmax, okFlag(d.MaxQ, lim.Qmax))
	fmt.Printf("  (max q at t = %.1f s)\n", d.MaxQt)

	fmt.Println("=== Atmosphere boundary (94 km) ===")
	if d.CrossUpStage > 0 {
		fmt.Printf("  crossing UP   : t=%.1f s, H=%.1f km, during stage %d (§4.1 expects stage 2)\n", d.CrossUpTime, d.CrossUpH/1000, d.CrossUpStage)
	} else {
		fmt.Println("  crossing UP   : not reached")
	}
	if d.CrossDownTime > 0 {
		fmt.Printf("  crossing DOWN : t=%.1f s\n", d.CrossDownTime)
	}

	fmt.Println("=== Terminal parameters ===")
	fmt.Printf("  burnout (t=%.1f s): V=%.0f m/s, H=%.1f km, θ=%.1f deg\n", d.BurnoutT, d.BurnoutV, d.BurnoutH/1000, d.BurnoutTheta)
	fmt.Printf("  apogee           : H=%.1f km at t=%.1f s\n", d.ApogeeH/1000, d.ApogeeT)
	fmt.Printf("  impact           : range=%.1f km at t=%.1f s\n", d.ImpactRange/1000, d.ImpactT)

	// Note any aero parts served by a fallback (CFD case not present yet).
	for _, p := range []string{"all", "stage2up", "stage3up", "head"} {
		if used := at.Resolved(p); used != "" && used != p {
			fmt.Printf("  note: aero part %q not in data; using %q\n", p, used)
		}
	}
}
