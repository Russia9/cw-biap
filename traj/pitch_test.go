package traj

import (
	"math"
	"strings"
	"testing"
)

func TestCosinePitchSegmentsChainContinuously(t *testing.T) {
	p := PitchProgram{
		TVert: 10,
		Segments: []PitchSegment{
			{TEnd: 25, Theta: deg(80), Shape: ShapeCos, K: 3},
			{TEnd: 66.4, Theta: deg(72), Shape: ShapeCos, K: 3},
		},
	}

	assertClose(t, "vertical hold", p.Theta(10), math.Pi/2, 1e-12)
	assertClose(t, "first endpoint", p.Theta(25), deg(80), 1e-12)
	assertClose(t, "second start", p.Theta(math.Nextafter(25, math.Inf(1))), deg(80), 1e-12)
	assertClose(t, "second endpoint", p.Theta(66.4), deg(72), 1e-12)

	assertClose(t, "vertical rate", p.Rate(10), 0, 1e-12)
	assertClose(t, "first endpoint rate", p.Rate(25), 0, 1e-12)
	assertClose(t, "final endpoint rate", p.Rate(66.4), 0, 1e-12)
}

func TestCosinePitchUsesLocalSegmentFormula(t *testing.T) {
	p := PitchProgram{
		TVert: 10,
		Segments: []PitchSegment{
			{TEnd: 20, Theta: deg(80), Shape: ShapeCos, K: 2},
			{TEnd: 30, Theta: deg(60), Shape: ShapeCos, K: 2},
		},
	}

	a, b := deg(80), deg(60)
	s := (25.0 - 20.0) / (30.0 - 20.0)
	want := (a+b)/2 + (a-b)/2*math.Cos(math.Pi*math.Pow(s, 2))
	assertClose(t, "second segment midpoint", p.Theta(25), want, 1e-12)
}

func TestCosinePitchNormalizesLowExponentForSmoothStart(t *testing.T) {
	p := PitchProgram{
		TVert: 10,
		Segments: []PitchSegment{
			{TEnd: 20, Theta: deg(80), Shape: ShapeCos, K: 0.5},
		},
	}

	a, b := math.Pi/2, deg(80)
	s := (15.0 - 10.0) / (20.0 - 10.0)
	want := (a+b)/2 + (a-b)/2*math.Cos(math.Pi*s)
	assertClose(t, "midpoint uses minimum smooth exponent", p.Theta(15), want, 1e-12)
	assertClose(t, "rate at segment start", p.Rate(10), 0, 1e-12)
}

func TestExponentialPitchSegmentsChainSmoothly(t *testing.T) {
	p := PitchProgram{
		TVert: 10,
		Segments: []PitchSegment{
			{TEnd: 25, Theta: deg(80), Shape: ShapeExp, K: 3},
			{TEnd: 66.4, Theta: deg(72), Shape: ShapeExp, K: -2},
		},
	}

	assertClose(t, "vertical hold", p.Theta(10), math.Pi/2, 1e-12)
	assertClose(t, "first endpoint", p.Theta(25), deg(80), 1e-12)
	assertClose(t, "second start", p.Theta(math.Nextafter(25, math.Inf(1))), deg(80), 1e-12)
	assertClose(t, "second endpoint", p.Theta(66.4), deg(72), 1e-12)

	assertClose(t, "vertical rate", p.Rate(10), 0, 1e-12)
	assertClose(t, "first endpoint rate", p.Rate(25), 0, 1e-12)
	assertClose(t, "final endpoint rate", p.Rate(66.4), 0, 1e-12)
}

func TestExponentialPitchZeroKUsesSmootherstep(t *testing.T) {
	p := PitchProgram{
		TVert: 10,
		Segments: []PitchSegment{
			{TEnd: 20, Theta: deg(80), Shape: ShapeExp, K: 0},
		},
	}

	a, b := math.Pi/2, deg(80)
	s := (15.0 - 10.0) / (20.0 - 10.0)
	u := s * s * s * (s*(s*6-15) + 10)
	assertClose(t, "midpoint uses smootherstep limit", p.Theta(15), a+(b-a)*u, 1e-12)
	assertClose(t, "rate at segment start", p.Rate(10), 0, 1e-12)
	assertClose(t, "rate at segment end", p.Rate(20), 0, 1e-12)
}

func TestPitchProgramValidateRejectsNonIncreasingSegments(t *testing.T) {
	p := PitchProgram{
		TVert: 10,
		Segments: []PitchSegment{
			{TEnd: 20, Theta: deg(80), Shape: ShapeCos, K: 3},
			{TEnd: 20, Theta: deg(72), Shape: ShapeCos, K: 3},
		},
	}

	err := p.Validate()
	if err == nil {
		t.Fatal("Validate() returned nil for non-increasing segment times")
	}
	if !strings.Contains(err.Error(), "segment 1") {
		t.Fatalf("Validate() error = %q, want segment index", err)
	}
}

func assertClose(t *testing.T, name string, got, want, tol float64) {
	t.Helper()
	if math.Abs(got-want) > tol {
		t.Fatalf("%s: got %.17g, want %.17g", name, got, want)
	}
}
