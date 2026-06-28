package traj

import (
	"encoding/csv"
	"fmt"
	"math"
	"os"
	"sort"
	"strconv"
)

// machGroup holds the coefficient curves at one Mach number, sorted by angle of
// attack. Each Mach may carry a different set of α points (the CFD grid is
// ragged and still growing), so coefficients are interpolated separably: over α
// within a Mach, then over Mach.
type machGroup struct {
	mach       float64
	alpha      []float64 // ascending [deg]
	cd, cl, cm []float64
}

type partTable struct {
	groups []machGroup // ascending by mach
}

// AeroTable provides Cd, Cl, CmPitch lookups per rocket part. All coefficients
// share the openfoam reference (Aref, Lref).
type AeroTable struct {
	parts map[string]*partTable
}

// fallbacks lists, for each flight part, the substitute part to use when the
// part's own CFD data is not present yet (the sweep is still running). The
// warhead-alone "head" case falls back to the stage-3 stack.
var fallbacks = map[string][]string{
	"head":     {"head", "stage3up", "stage2up", "all"},
	"stage3up": {"stage3up", "stage2up", "all"},
	"stage2up": {"stage2up", "all"},
	"all":      {"all"},
}

// LoadAero parses an averages.csv (openfoam/results/averages.csv) into an
// AeroTable. Columns are located by header name so added columns/rows are
// tolerated.
func LoadAero(path string) (*AeroTable, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	r := csv.NewReader(f)
	r.FieldsPerRecord = -1
	records, err := r.ReadAll()
	if err != nil {
		return nil, err
	}
	if len(records) < 2 {
		return nil, fmt.Errorf("aero: %s has no data rows", path)
	}

	col := map[string]int{}
	for i, name := range records[0] {
		col[name] = i
	}
	need := []string{"part", "Ma", "alpha", "Cd_mean", "Cl_mean", "CmPitch_mean"}
	for _, n := range need {
		if _, ok := col[n]; !ok {
			return nil, fmt.Errorf("aero: %s missing column %q", path, n)
		}
	}

	type sample struct{ alpha, cd, cl, cm float64 }
	// part -> mach -> samples
	raw := map[string]map[float64][]sample{}
	atof := func(s string) float64 { v, _ := strconv.ParseFloat(s, 64); return v }

	for _, rec := range records[1:] {
		part := rec[col["part"]]
		ma := atof(rec[col["Ma"]])
		s := sample{
			alpha: atof(rec[col["alpha"]]),
			cd:    atof(rec[col["Cd_mean"]]),
			cl:    atof(rec[col["Cl_mean"]]),
			cm:    atof(rec[col["CmPitch_mean"]]),
		}
		if raw[part] == nil {
			raw[part] = map[float64][]sample{}
		}
		raw[part][ma] = append(raw[part][ma], s)
	}

	at := &AeroTable{parts: map[string]*partTable{}}
	for part, byMach := range raw {
		pt := &partTable{}
		for mach, samples := range byMach {
			sort.Slice(samples, func(i, j int) bool { return samples[i].alpha < samples[j].alpha })
			g := machGroup{mach: mach}
			for _, s := range samples {
				g.alpha = append(g.alpha, s.alpha)
				g.cd = append(g.cd, s.cd)
				g.cl = append(g.cl, s.cl)
				g.cm = append(g.cm, s.cm)
			}
			pt.groups = append(pt.groups, g)
		}
		sort.Slice(pt.groups, func(i, j int) bool { return pt.groups[i].mach < pt.groups[j].mach })
		at.parts[part] = pt
	}
	return at, nil
}

// resolve returns the table actually used for a requested part, following the
// fallback chain, and whether a fallback was needed.
func (a *AeroTable) resolve(part string) (*partTable, string) {
	for _, p := range fallbacks[part] {
		if pt, ok := a.parts[p]; ok {
			return pt, p
		}
	}
	return nil, ""
}

// Has reports whether the requested part (or a fallback) has data.
func (a *AeroTable) Has(part string) bool {
	pt, _ := a.resolve(part)
	return pt != nil
}

// Resolved returns which part key is actually used for a requested part.
func (a *AeroTable) Resolved(part string) string {
	_, used := a.resolve(part)
	return used
}

// Coeffs returns drag, lift and pitch-moment coefficients for a part at the
// given Mach and angle of attack [deg]. Cd is even in α; Cl and CmPitch are odd,
// so they carry the sign of α. Mach and |α| are clamped to the part's
// data-driven bounds. Missing data (no part, no fallback) yields zeros.
func (a *AeroTable) Coeffs(part string, mach, alphaDeg float64) (cd, cl, cm float64) {
	pt, _ := a.resolve(part)
	if pt == nil || len(pt.groups) == 0 {
		return 0, 0, 0
	}

	sign := 1.0
	if alphaDeg < 0 {
		sign = -1.0
	}
	aabs := math.Abs(alphaDeg)

	// Evaluate each Mach group's α-curves at aabs, then interpolate over Mach.
	machs := make([]float64, len(pt.groups))
	cds := make([]float64, len(pt.groups))
	cls := make([]float64, len(pt.groups))
	cms := make([]float64, len(pt.groups))
	for i, g := range pt.groups {
		machs[i] = g.mach
		cds[i] = interp1(g.alpha, g.cd, aabs)
		cls[i] = interp1(g.alpha, g.cl, aabs)
		cms[i] = interp1(g.alpha, g.cm, aabs)
	}
	cd = interp1(machs, cds, mach)
	cl = interp1(machs, cls, mach) * sign
	cm = interp1(machs, cms, mach) * sign
	return cd, cl, cm
}

// interp1 does 1-D linear interpolation of ys over ascending xs, clamping to
// the end values outside [xs[0], xs[last]]. Single-point curves return that
// point.
func interp1(xs, ys []float64, x float64) float64 {
	n := len(xs)
	if n == 0 {
		return 0
	}
	if n == 1 || x <= xs[0] {
		return ys[0]
	}
	if x >= xs[n-1] {
		return ys[n-1]
	}
	hi := sort.SearchFloat64s(xs, x)
	lo := hi - 1
	t := (x - xs[lo]) / (xs[hi] - xs[lo])
	return ys[lo] + t*(ys[hi]-ys[lo])
}
