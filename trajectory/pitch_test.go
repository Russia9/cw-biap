package traj

import (
	"math"
	"testing"
)

func deg(p Pitch, t float64) float64 { return p.Pitch(t) * r2d }

// slopeDeg is dϑ/dt in deg/s by central difference. Keep t at least 1e-4 away
// from a joint so both samples fall inside the same arc.
func slopeDeg(p Pitch, t float64) float64 {
	const e = 1e-6
	return (p.Pitch(t+e) - p.Pitch(t-e)) / (2 * e) * r2d
}

func near(t *testing.T, name string, got, want, tol float64) {
	t.Helper()
	if math.Abs(got-want) > tol {
		t.Errorf("%s: got %.6f, want %.6f (tol %g)", name, got, want, tol)
	}
}

func TestHermiteEndpointsAndSlopes(t *testing.T) {
	p := Pitch{ThetaDegStart: 90, TStart: 5, Segments: []PitchSegment{
		{TEnd: 15, Shape: PitchShapeHermite, K: -2.5, ThetaDeg: 70},
		{TEnd: 30, Shape: PitchShapeHermite, K: -0.8, ThetaDeg: 45},
	}}
	near(t, "theta at t_start", deg(p, 5+1e-9), 90, 1e-6)
	near(t, "theta at end of arc 1", deg(p, 15), 70, 1e-9)
	near(t, "theta at end of arc 2", deg(p, 30), 45, 1e-9)
	// the program holds ThetaDegStart before TStart, so the first arc starts flat
	near(t, "slope at start of arc 1", slopeDeg(p, 5+1e-4), 0, 1e-2)
	// an arc exits at its own K and the next one enters at that same value
	near(t, "slope at exit of arc 1", slopeDeg(p, 15-1e-4), -2.5, 1e-2)
	near(t, "slope at entry of arc 2", slopeDeg(p, 15+1e-4), -2.5, 1e-2)
	near(t, "slope at exit of arc 2", slopeDeg(p, 30-1e-4), -0.8, 1e-2)
}

func TestHermiteJointIsC1(t *testing.T) {
	p := Pitch{ThetaDegStart: 90, TStart: 5, Segments: []PitchSegment{
		{TEnd: 20, Shape: PitchShapeHermite, K: -1.7, ThetaDeg: 60},
		{TEnd: 40, Shape: PitchShapeHermite, K: -0.3, ThetaDeg: 30},
		{TEnd: 60, Shape: PitchShapeHermite, K: 0.4, ThetaDeg: 35}, // a pull-back
	}}
	for _, j := range []float64{20, 40} {
		near(t, "position continuous", deg(p, j-1e-9), deg(p, j+1e-9), 1e-6)
		near(t, "slope continuous", slopeDeg(p, j-1e-4), slopeDeg(p, j+1e-4), 1e-2)
	}
}

func TestCosIntoHermiteIsC1(t *testing.T) {
	// a cos arc always ends flat, so the hermite arc after it must start flat
	p := Pitch{ThetaDegStart: 90, TStart: 5, Segments: []PitchSegment{
		{TEnd: 20, Shape: PitchShapeCos, K: 3, ThetaDeg: 60},
		{TEnd: 40, Shape: PitchShapeHermite, K: -1.2, ThetaDeg: 30},
	}}
	near(t, "cos exit slope", slopeDeg(p, 20-1e-4), 0, 1e-2)
	near(t, "hermite entry slope after cos", slopeDeg(p, 20+1e-4), 0, 1e-2)
	near(t, "hermite exit slope", slopeDeg(p, 40-1e-4), -1.2, 1e-2)
}

func TestCosUnchangedBySignature(t *testing.T) {
	// the cos arc ignores the new slope argument: pin it to its closed form
	p := Pitch{ThetaDegStart: 90, TStart: 5, Segments: []PitchSegment{
		{TEnd: 25, Shape: PitchShapeCos, K: 2.5, ThetaDeg: 50},
	}}
	for _, tt := range []float64{6, 10, 15, 20, 24.9} {
		x := (tt - 5) / 20
		want := (90+50)/2.0 + (90-50)/2.0*math.Cos(math.Pi*math.Pow(x, 2.5))
		near(t, "cos closed form", deg(p, tt), want, 1e-9)
	}
}

func TestHoldPastLastArc(t *testing.T) {
	p := Pitch{ThetaDegStart: 90, TStart: 5, Segments: []PitchSegment{
		{TEnd: 20, Shape: PitchShapeHermite, K: -2.0, ThetaDeg: 40},
	}}
	near(t, "held angle", deg(p, 500), 40, 1e-12)
	near(t, "held slope", slopeDeg(p, 500), 0, 1e-12)
}
