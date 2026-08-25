package atmosphere

import (
	"math"
	"testing"
)

// The GOST 4401-81 conformance test. It checks the model against the standard
// it implements, in four ways:
//
//  1. Anchor reproduction: at the geometric altitude of each table row's
//     geopotential anchor, the model must return the anchored T and P.
//  2. Printed-table spot checks: values read manually from the printed
//     GOST 4401-81 parameter tables (geometric-altitude rows, 2026-08 audit).
//     These pinned down the two defects fixed then: layer selection by
//     geometric height, and the 85 km row's lapse rate and pressure anchor.
//  3. Continuity: T and p must be continuous across every layer boundary
//     (the standard's piecewise-linear T(H') is continuous by construction).
//  4. The 94 km handoff into the vacuum branch must be temperature-continuous.
func TestGOSTAtmosphere(t *testing.T) {
	// geomOf inverts H' = R·h/(R+h): the geometric altitude of a geopotential one.
	geomOf := func(H float64) float64 { return REarth * H / (REarth - H) }

	t.Run("anchors", func(t *testing.T) {
		// The GOST 4401-81 layer anchors (geopotential height, K, Pa). The
		// 85000 anchor is the barometric continuation of the 71000 layer,
		// confirmed against the printed tables (see the table in atmosphere.go).
		anchors := []struct{ H, T, P float64 }{
			{0, 288.150, 101325},
			{11000, 216.650, 22632.0},
			{20000, 216.650, 5474.87},
			{32000, 228.65, 868.014},
			{47000, 270.65, 110.906},
			{51000, 270.65, 66.9384},
			{71000, 214.65, 3.95639},
			{85000, 186.65, 0.363409},
		}
		for _, a := range anchors {
			h := geomOf(a.H)
			_, _, p, T, _, _ := Atmosphere(h)
			if math.Abs(T-a.T) > 1e-6 {
				t.Errorf("T at H'=%.0f (h=%.1f): got %.6f K, GOST %.3f K", a.H, h, T, a.T)
			}
			if math.Abs(p-a.P)/a.P > 1e-3 {
				t.Errorf("p at H'=%.0f (h=%.1f): got %.6g Pa, GOST %.6g Pa", a.H, h, p, a.P)
			}
		}
	})

	t.Run("sea level", func(t *testing.T) {
		_, rho, _, _, _, a := Atmosphere(0)
		if math.Abs(rho-1.225) > 1e-3 {
			t.Errorf("rho(0) = %.6f, GOST 1.225 kg/m³", rho)
		}
		if math.Abs(a-340.294) > 0.01 {
			t.Errorf("a(0) = %.4f, GOST 340.294 m/s", a)
		}
	})

	t.Run("printed values", func(t *testing.T) {
		// Read manually from the printed GOST 4401-81 tables (rows indexed by
		// geometric altitude) during the 2026-08 audit. A zero means the value
		// was not read. The T rows discriminate the layer-selection rule; the
		// p rows above 85 km discriminate the isothermal layer and its anchor
		// (the pre-audit code was ~6 % low there).
		printed := []struct{ h, T, P float64 }{
			{20050, 216.650, 0},
			{32100, 228.589, 0},
			{47200, 270.236, 0},
			{71400, 215.751, 0},
			{85000, 0, 0.445710},
			{85500, 0, 0.4080},
			{86000, 0, 0.373380},
			{86500, 0, 0.341546},
			{88000, 186.650, 0.261501},
			{90000, 186.650, 0.183140},
			{92000, 186.650, 0.128308},
		}
		for _, row := range printed {
			_, _, p, T, _, _ := Atmosphere(row.h)
			if row.T != 0 && math.Abs(T-row.T) > 0.005 {
				t.Errorf("T at h=%.0f: got %.4f K, printed %.3f K", row.h, T, row.T)
			}
			if row.P != 0 && math.Abs(p-row.P)/row.P > 1e-3 {
				t.Errorf("p at h=%.0f: got %.6g Pa, printed %.6g Pa", row.h, p, row.P)
			}
		}
	})

	t.Run("continuity", func(t *testing.T) {
		// Scan ±30 m around every place a layer switch could occur — the table's
		// geopotential anchors and their geometric equivalents — and require step
		// continuity. Thresholds sit well above one 1 m step of the steepest legal
		// gradient (0.0065 K, ~2.2e-4 relative p) and well below the defects they
		// exist to catch (0.06–1.14 K, 6e-2 relative p).
		for _, b := range []float64{11000, 20000, 32000, 47000, 51000, 71000, 85000} {
			for _, base := range []float64{b, geomOf(b)} {
				for h := base - 30; h < base+30; h++ {
					_, _, p0, T0, _, _ := Atmosphere(h)
					_, _, p1, T1, _, _ := Atmosphere(h + 1)
					if math.Abs(T1-T0) > 0.01 {
						t.Fatalf("T discontinuity near %.0f m: T(%.1f)=%.4f K, T(%.1f)=%.4f K", base, h, T0, h+1, T1)
					}
					if rel := math.Abs(p1-p0) / p0; rel > 5e-4 {
						t.Fatalf("p discontinuity near %.0f m: p(%.1f)=%.6g, p(%.1f)=%.6g (rel %.2g)", base, h, p0, h+1, p1, rel)
					}
				}
			}
		}
	})

	t.Run("vacuum handoff", func(t *testing.T) {
		_, _, _, Tbelow, _, _ := Atmosphere(HBoundary - 0.001)
		_, _, _, Tabove, _, _ := Atmosphere(HBoundary + 0.001)
		if math.Abs(Tabove-Tbelow) > 0.01 {
			t.Errorf("T steps across the 94 km boundary: %.4f K below, %.4f K above", Tbelow, Tabove)
		}
	})
}
