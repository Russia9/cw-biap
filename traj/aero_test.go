package traj

import (
	"math"
	"testing"
)

func almost(a, b float64) bool { return math.Abs(a-b) < 1e-12 }

func TestInterp1(t *testing.T) {
	xs := []float64{1, 2, 4}
	ys := []float64{10, 20, 40}
	cases := []struct{ x, want float64 }{
		{0, 10},   // clamp below
		{1, 10},   // left endpoint
		{1.5, 15}, // interior
		{3, 30},   // interior, uneven spacing
		{4, 40},   // right endpoint
		{9, 40},   // clamp above
	}
	for _, c := range cases {
		if got := interp1(xs, ys, c.x); !almost(got, c.want) {
			t.Errorf("interp1(%v) = %v, want %v", c.x, got, c.want)
		}
	}
	if got := interp1([]float64{7}, []float64{3.5}, 100); !almost(got, 3.5) {
		t.Errorf("single-point curve: got %v, want 3.5", got)
	}
	if got := interp1(nil, nil, 1); got != 0 {
		t.Errorf("empty curve: got %v, want 0", got)
	}
}

func loadFixture(t *testing.T) *AeroTable {
	t.Helper()
	at, err := LoadAero("testdata/aero_fixture.csv")
	if err != nil {
		t.Fatal(err)
	}
	return at
}

func TestLoadAeroFixture(t *testing.T) {
	at := loadFixture(t)
	// Interior bilinear point for "all": Mach 1.2 is halfway between the 0.4 and
	// 2.0 groups, α=5° is a grid line, so Cd = (0.32+0.70)/2.
	cd, cl, cm := at.Coeffs("all", 1.2, 5)
	if !almost(cd, 0.51) || !almost(cl, 0.50) || !almost(cm, -0.65) {
		t.Errorf("Coeffs(all, 1.2, 5) = %v %v %v, want 0.51 0.50 -0.65", cd, cl, cm)
	}
	// α between grid lines within one Mach group: α=7.5° at Mach 0.4.
	cd, _, _ = at.Coeffs("all", 0.4, 7.5)
	if !almost(cd, 0.36) {
		t.Errorf("Coeffs(all, 0.4, 7.5) cd = %v, want 0.36", cd)
	}
}

func TestCoeffsAlphaSymmetry(t *testing.T) {
	at := loadFixture(t)
	cdP, clP, cmP := at.Coeffs("all", 2.0, 5)
	cdN, clN, cmN := at.Coeffs("all", 2.0, -5)
	if cdN != cdP {
		t.Errorf("Cd must be even in α: %v vs %v", cdP, cdN)
	}
	if clN != -clP || cmN != -cmP {
		t.Errorf("Cl/Cm must be odd in α: (%v %v) vs (%v %v)", clP, cmP, clN, cmN)
	}
}

func TestCoeffsClamping(t *testing.T) {
	at := loadFixture(t)
	// Above the Mach range: clamps to the Mach-2 group.
	cd, _, _ := at.Coeffs("all", 8.0, 0)
	if !almost(cd, 0.60) {
		t.Errorf("Mach clamp high: cd = %v, want 0.60", cd)
	}
	// |α| above the α range: clamps to the 10° value, sign preserved.
	_, cl, _ := at.Coeffs("all", 2.0, -25)
	if !almost(cl, -1.30) {
		t.Errorf("alpha clamp: cl = %v, want -1.30", cl)
	}
}

func TestFallbackChain(t *testing.T) {
	at := loadFixture(t)
	// The fixture has "all" and "stage3up" only.
	cases := []struct{ part, want string }{
		{"all", "all"},
		{"stage2up", "all"},      // stage2up -> all
		{"stage3up", "stage3up"}, // own data present
		{"head", "stage3up"},     // head -> stage3up
	}
	for _, c := range cases {
		if got := at.Resolved(c.part); got != c.want {
			t.Errorf("Resolved(%q) = %q, want %q", c.part, got, c.want)
		}
	}
	// A fallback must serve the substitute's coefficients.
	cdH, _, _ := at.Coeffs("head", 15, 0)
	cdS, _, _ := at.Coeffs("stage3up", 15, 0)
	if cdH != cdS {
		t.Errorf("head must fall back to stage3up: %v vs %v", cdH, cdS)
	}
}

func TestZeroAero(t *testing.T) {
	at := ZeroAero()
	if cd, cl, cm := at.Coeffs("all", 3, 5); cd != 0 || cl != 0 || cm != 0 {
		t.Errorf("ZeroAero Coeffs = %v %v %v, want zeros", cd, cl, cm)
	}
	if got := at.Resolved("all"); got != "" {
		t.Errorf("ZeroAero Resolved = %q, want empty", got)
	}
}

func TestLoadAeroErrors(t *testing.T) {
	if _, err := LoadAero("testdata/does-not-exist.csv"); err == nil {
		t.Error("missing file: want error")
	}
}
