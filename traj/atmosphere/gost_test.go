package atmosphere

import (
	"math"
	"testing"
)

// The GOST 4401-81 audit instrument. It checks the model against the standard
// it claims to implement, in three ways:
//
//  1. Anchor reproduction: at the geometric altitude of each table row's
//     geopotential anchor, the model must return the published T and P.
//  2. Continuity: T and p must be continuous across every layer boundary
//     (the standard's piecewise-linear T(H') is continuous by construction).
//  3. The 94 km handoff into the vacuum branch must be temperature-continuous.
//
// Currently skipped: (2) and (3) fail on HEAD because the layer is selected by
// geometric h against geopotential anchors (atmosphere.go:67) and because the
// 85 km row's Beta disagrees with the 94 km row. Un-skip during the physics
// audit; after the fix this test stays green permanently.
func TestGOSTAtmosphere(t *testing.T) {
	t.Skip("audit instrument: un-skip in the physics-audit phase")

	// geomOf inverts H' = R·h/(R+h): the geometric altitude of a geopotential one.
	geomOf := func(H float64) float64 { return REarth * H / (REarth - H) }

	t.Run("anchors", func(t *testing.T) {
		// The published GOST 4401-81 layer anchors (geopotential height, K, Pa) —
		// the same values the implementation's table carries.
		anchors := []struct{ H, T, P float64 }{
			{0, 288.150, 101325},
			{11000, 216.650, 22632.0},
			{20000, 216.650, 5474.87},
			{32000, 228.65, 868.014},
			{47000, 270.65, 110.906},
			{51000, 270.65, 66.9384},
			{71000, 214.65, 3.95639},
			{85000, 186.65, 0.341546},
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
