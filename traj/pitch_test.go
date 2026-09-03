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
			{TEnd: 25, Val: deg(80), Shape: ShapeCos, K: 3},
			{TEnd: 66.4, Val: deg(72), Shape: ShapeCos, K: 3},
		},
	}

	assertClose(t, "vertical hold", p.Cmd(10, 0), math.Pi/2, 1e-12)
	assertClose(t, "first endpoint", p.Cmd(25, 0), deg(80), 1e-12)
	assertClose(t, "second start", p.Cmd(math.Nextafter(25, math.Inf(1)), 0), deg(80), 1e-12)
	assertClose(t, "second endpoint", p.Cmd(66.4, 0), deg(72), 1e-12)

	assertClose(t, "vertical rate", p.Rate(10, 0), 0, 1e-12)
	assertClose(t, "first endpoint rate", p.Rate(25, 0), 0, 1e-12)
	assertClose(t, "final endpoint rate", p.Rate(66.4, 0), 0, 1e-12)
}

func TestCosinePitchUsesLocalSegmentFormula(t *testing.T) {
	p := PitchProgram{
		TVert: 10,
		Segments: []PitchSegment{
			{TEnd: 20, Val: deg(80), Shape: ShapeCos, K: 2},
			{TEnd: 30, Val: deg(60), Shape: ShapeCos, K: 2},
		},
	}

	a, b := deg(80), deg(60)
	s := (25.0 - 20.0) / (30.0 - 20.0)
	want := (a+b)/2 + (a-b)/2*math.Cos(math.Pi*math.Pow(s, 2))
	assertClose(t, "second segment midpoint", p.Cmd(25, 0), want, 1e-12)
}

func TestCosinePitchNormalizesLowExponentForSmoothStart(t *testing.T) {
	p := PitchProgram{
		TVert: 10,
		Segments: []PitchSegment{
			{TEnd: 20, Val: deg(80), Shape: ShapeCos, K: 0.5},
		},
	}

	a, b := math.Pi/2, deg(80)
	s := (15.0 - 10.0) / (20.0 - 10.0)
	want := (a+b)/2 + (a-b)/2*math.Cos(math.Pi*s)
	assertClose(t, "midpoint uses minimum smooth exponent", p.Cmd(15, 0), want, 1e-12)
	assertClose(t, "rate at segment start", p.Rate(10, 0), 0, 1e-12)
}

func TestExponentialPitchSegmentsChainSmoothly(t *testing.T) {
	p := PitchProgram{
		TVert: 10,
		Segments: []PitchSegment{
			{TEnd: 25, Val: deg(80), Shape: ShapeExp, K: 3},
			{TEnd: 66.4, Val: deg(72), Shape: ShapeExp, K: -2},
		},
	}

	assertClose(t, "vertical hold", p.Cmd(10, 0), math.Pi/2, 1e-12)
	assertClose(t, "first endpoint", p.Cmd(25, 0), deg(80), 1e-12)
	assertClose(t, "second start", p.Cmd(math.Nextafter(25, math.Inf(1)), 0), deg(80), 1e-12)
	assertClose(t, "second endpoint", p.Cmd(66.4, 0), deg(72), 1e-12)

	assertClose(t, "vertical rate", p.Rate(10, 0), 0, 1e-12)
	assertClose(t, "first endpoint rate", p.Rate(25, 0), 0, 1e-12)
	assertClose(t, "final endpoint rate", p.Rate(66.4, 0), 0, 1e-12)
}

func TestExponentialPitchZeroKUsesSmootherstep(t *testing.T) {
	p := PitchProgram{
		TVert: 10,
		Segments: []PitchSegment{
			{TEnd: 20, Val: deg(80), Shape: ShapeExp, K: 0},
		},
	}

	a, b := math.Pi/2, deg(80)
	s := (15.0 - 10.0) / (20.0 - 10.0)
	u := s * s * s * (s*(s*6-15) + 10)
	assertClose(t, "midpoint uses smootherstep limit", p.Cmd(15, 0), a+(b-a)*u, 1e-12)
	assertClose(t, "rate at segment start", p.Rate(10, 0), 0, 1e-12)
	assertClose(t, "rate at segment end", p.Rate(20, 0), 0, 1e-12)
}

func TestPitchProgramValidateRejectsNonIncreasingSegments(t *testing.T) {
	p := PitchProgram{
		TVert: 10,
		Segments: []PitchSegment{
			{TEnd: 20, Val: deg(80), Shape: ShapeCos, K: 3},
			{TEnd: 20, Val: deg(72), Shape: ShapeCos, K: 3},
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

func TestAlphaFramedProgramHoldsCommandedAlpha(t *testing.T) {
	// Two arcs chained at the same value: a flat α plateau, which is exactly
	// what a ϑ-framed program cannot express (arcRate is 0 at every joint, so
	// α̇ = −θ̇ ≠ 0 there).
	p := PitchProgram{
		TVert: 10,
		Segments: []PitchSegment{
			{TEnd: 25, Val: deg(-9.5), Shape: ShapeCos, Frame: FrameAlpha, K: 1.1},
			{TEnd: 66.4, Val: deg(-9.5), Shape: ShapeCos, Frame: FrameAlpha, K: 1.1},
		},
	}

	// The flight path sweeps through the whole powered range; α must not care.
	for _, theta := range []float64{deg(90), deg(45), deg(20), deg(-5)} {
		for _, tt := range []float64{25.0001, 30, 50, 66.4} {
			got := p.Cmd(tt, theta) - theta
			assertClose(t, "held α", got, deg(-9.5), 1e-12)
		}
	}

	// ϑ̇ = θ̇ + α̇, and α̇ = 0 on the plateau.
	assertClose(t, "plateau rate follows θ̇", p.Rate(50, deg(1.7)), deg(1.7), 1e-12)
	assertClose(t, "joint rate follows θ̇", p.Rate(25, deg(1.7)), deg(1.7), 1e-12)
	assertClose(t, "vertical hold rate", p.Rate(5, deg(1.7)), 0, 1e-12)
}

func TestAlphaFramedFirstArcEntersFromZero(t *testing.T) {
	// At the vertical-hold exit the rocket still flies straight up, so ϑ = 90°
	// and α = 0 describe the same attitude and the first arc starts from 0.
	p := PitchProgram{
		TVert: 10,
		Segments: []PitchSegment{
			{TEnd: 20, Val: deg(-8), Shape: ShapeCos, Frame: FrameAlpha, K: 1},
		},
	}

	theta := deg(90)
	assertClose(t, "entry α", p.Cmd(10.0001, theta)-theta, 0, 1e-6)

	a, b := 0.0, deg(-8)
	s := (15.0 - 10.0) / (20.0 - 10.0)
	want := (a+b)/2 + (a-b)/2*math.Cos(math.Pi*s)
	assertClose(t, "midpoint α", p.Cmd(15, theta)-theta, want, 1e-12)
}

func TestFrameChangeUsesExplicitEntryAngle(t *testing.T) {
	entry := deg(50)
	p := PitchProgram{
		TVert: 10,
		Segments: []PitchSegment{
			{TEnd: 20, Val: deg(-8), Shape: ShapeCos, Frame: FrameAlpha, K: 1},
			{TEnd: 30, Val: deg(40), Shape: ShapeCos, Frame: FrameTheta, K: 1, Entry: &entry},
		},
	}

	if err := p.Validate(); err != nil {
		t.Fatalf("Validate() rejected an explicit entry angle: %v", err)
	}
	// The ϑ arc starts from the entry angle, not from the α value it follows.
	assertClose(t, "entry pitch", p.Cmd(20.0001, deg(30)), deg(50), 1e-6)
}

func TestSimulateLatchesFrameChangeEntryFromState(t *testing.T) {
	// Stage 1 steers α, stage 2 steers ϑ. Continuity across the joint needs
	// ϑ = θ + α there, which is only known once the state is.
	r := Rocket{
		Stages: []Stage{{BurnTime: 20}, {BurnTime: 20}},
		Pitch: PitchProgram{
			TVert: 5,
			Segments: []PitchSegment{
				{TEnd: 20, Val: deg(-8), Shape: ShapeCos, Frame: FrameAlpha, K: 1},
				{TEnd: 40, Val: deg(30), Shape: ShapeCos, Frame: FrameTheta, K: 1},
			},
		},
	}

	// A state with θ = 45°, at the moment the frame changes.
	state := []float64{100, 100, 0, 1000, 500}
	theta := FlightAngle(state)
	assertClose(t, "test state θ", theta, deg(45), 1e-12)

	r.latchFrameEntry(20, state)
	if r.Pitch.Segments[1].Entry == nil {
		t.Fatal("latchFrameEntry left the frame change unresolved")
	}
	assertClose(t, "latched entry ϑ", *r.Pitch.Segments[1].Entry, deg(45-8), 1e-12)

	// ϑ is now continuous across the joint from either side.
	before := r.Pitch.Cmd(20, theta)
	after := r.Pitch.Cmd(math.Nextafter(20, math.Inf(1)), theta)
	assertClose(t, "ϑ continuous across the frame change", after, before, 1e-9)
}

func assertClose(t *testing.T, name string, got, want, tol float64) {
	t.Helper()
	if math.Abs(got-want) > tol {
		t.Fatalf("%s: got %.17g, want %.17g", name, got, want)
	}
}

// TestHermitePitchSegmentsAreC1 is the property the shape exists for: unlike
// cos and exp, a Hermite chain does NOT pin ϑ̇ = 0 at its joints, so a ϑ-framed
// program can track θ̇ across a joint and hold an α plateau.
func TestHermitePitchSegmentsAreC1(t *testing.T) {
	p := PitchProgram{
		TVert: 10,
		Segments: []PitchSegment{
			{TEnd: 25, Val: deg(80), Shape: ShapeHermite, K: deg(-0.8)},
			{TEnd: 66.4, Val: deg(72), Shape: ShapeHermite, K: deg(-0.3)},
		},
	}

	assertClose(t, "vertical hold", p.Cmd(10, 0), math.Pi/2, 1e-12)
	assertClose(t, "first endpoint", p.Cmd(25, 0), deg(80), 1e-12)
	assertClose(t, "second endpoint", p.Cmd(66.4, 0), deg(72), 1e-12)

	// The terminal rate is the arc's own K...
	assertClose(t, "first endpoint rate", p.Rate(25, 0), deg(-0.8), 1e-12)
	assertClose(t, "final endpoint rate", p.Rate(66.4, 0), deg(-0.3), 1e-12)
	// ...and the next arc enters at exactly that rate, which is C¹.
	after := math.Nextafter(25, math.Inf(1))
	assertClose(t, "joint is C1 in value", p.Cmd(after, 0), deg(80), 1e-9)
	assertClose(t, "joint is C1 in rate", p.Rate(after, 0), deg(-0.8), 1e-9)
	// The first arc enters at rate 0, matching the vertical hold it leaves.
	assertClose(t, "entry rate from vertical hold", p.Rate(math.Nextafter(10, math.Inf(1)), 0), 0, 1e-9)
}

// TestHermiteRateMatchesFiniteDifference guards the analytic derivative, which
// feeds the §4.4 pitch-rate check directly.
func TestHermiteRateMatchesFiniteDifference(t *testing.T) {
	p := PitchProgram{
		TVert: 5,
		Segments: []PitchSegment{
			{TEnd: 30, Val: deg(70), Shape: ShapeHermite, K: deg(-1.2)},
			{TEnd: 60, Val: deg(40), Shape: ShapeHermite, K: deg(-0.5)},
		},
	}
	const h = 1e-6
	for _, tt := range []float64{7, 12.5, 20, 29.9, 31, 45, 59.5} {
		fd := (p.Cmd(tt+h, 0) - p.Cmd(tt-h, 0)) / (2 * h)
		assertClose(t, "rate at t", p.Rate(tt, 0), fd, 1e-6)
	}
}

// TestHermiteWithZeroRatesIsRatePinned pins the degenerate case: K = 0 on every
// arc reproduces the joint behaviour of the older shapes, so the new shape is a
// strict generalization rather than a different law.
func TestHermiteWithZeroRatesIsRatePinned(t *testing.T) {
	p := PitchProgram{
		TVert: 0,
		Segments: []PitchSegment{
			{TEnd: 10, Val: deg(60), Shape: ShapeHermite, K: 0},
		},
	}
	assertClose(t, "endpoint rate", p.Rate(10, 0), 0, 1e-12)
	assertClose(t, "start rate", p.Rate(math.Nextafter(0, math.Inf(1)), 0), 0, 1e-9)
	// Midpoint of a zero-slope Hermite is the plain average (smoothstep at s=0.5).
	assertClose(t, "midpoint", p.Cmd(5, 0), (math.Pi/2+deg(60))/2, 1e-12)
}
