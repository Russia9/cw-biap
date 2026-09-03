package traj

import (
	"fmt"
	"math"
)

// Shape selects the interpolation law of a pitch arc (§4.3). Each stage's arcs
// may use a different shape, so the steering program is composed per stage.
type Shape int

const (
	// ShapeExp is the exponential front-loaded arc (the original program law).
	ShapeExp Shape = iota
	// ShapeCos is the chained half-cosine arc generalized from the C++
	// reference (traj-example/model.hpp).
	ShapeCos
	// ShapeHermite is the C¹ cubic Hermite arc, whose terminal *rate* is the
	// free parameter instead of a shape exponent. Unlike exp and cos it does
	// not pin ϑ̇ = 0 at its joints, which is what lets a ϑ-framed program hold
	// an α plateau (see the Frame doc below).
	ShapeHermite
)

// Frame selects the reference the arc's value is measured against.
//
// FrameTheta is the original law: the arc interpolates the inertial pitch ϑ
// directly, so ϑ_пр(t) is a pure function of time and the angle of attack
// α = ϑ_пр(t) − θ(t) is whatever the trajectory leaves over.
//
// FrameAlpha inverts that: the arc interpolates α, and the commanded pitch is
// ϑ = θ + α_пр(t). A constant α is then exactly representable.
//
// With the exp and cos shapes the ϑ frame cannot do that — both pin ϑ̇ = 0 at
// their endpoints (see arcRate), so a ϑ-framed program is forced off any α
// plateau at each joint by α̇ = −θ̇. That is why the α frame was adopted.
// ShapeHermite lifts the restriction: its terminal rate is a free parameter, so
// a ϑ-framed Hermite program can track θ̇ across a joint and hold α after all.
type Frame int

const (
	// FrameTheta interpolates the inertial pitch ϑ (the original program law).
	FrameTheta Frame = iota
	// FrameAlpha interpolates the angle of attack α, relative to the flight path.
	FrameAlpha
)

// Default shape exponents when a config arc leaves k unset.
const (
	defaultKExp = 3.0 // exp: >0 front-loads the turn, 0 ⇒ smootherstep-only
	defaultKCos = 1.1 // cos: power on normalized time (C++ reference value)
	minKCos     = 1.0 // lower values make visually sharp, non-smooth starts
	// hermite: K is not an exponent at all but the arc's terminal rate [rad/s].
	// 0 reproduces the rate-pinned joint of the other two shapes.
	defaultKHermite = 0.0
)

// PitchSegment is one arc of the pitch program. It interpolates from its entry
// value (the previous segment's terminal value, or the frame's natural value at
// the vertical-hold exit) to Val, reaching Val exactly at TEnd. Val is an
// inertial pitch ϑ or an angle of attack α according to Frame.
type PitchSegment struct {
	TEnd  float64 // absolute segment end time [s]
	Val   float64 // terminal value [rad]: ϑ when FrameTheta, α when FrameAlpha
	Shape Shape
	Frame Frame
	// K is the shape exponent for exp and cos (cos values below minKCos are
	// normalized). For ShapeHermite it is instead the arc's TERMINAL RATE
	// [rad/s] — ϑ̇ when FrameTheta, α̇ when FrameAlpha — and the arc's entry rate
	// is the previous arc's K, which is what makes the chain C¹.
	K float64
	// Entry overrides the chained entry value [rad]. Required on the first arc
	// after a frame change, where the previous arc's terminal value is measured
	// against a different reference and so cannot be chained.
	Entry *float64
}

// PitchProgram is the programmed pitch ϑ_пр(t) [rad] on the active leg: a
// vertical hold of TVert seconds at 90°, then the chained arcs in Segments
// (ascending TEnd). Past the last segment it holds the final value (the program
// is only used while powered).
type PitchProgram struct {
	TVert    float64
	Segments []PitchSegment
}

// entryOf returns the value an arc starts from: an explicit Entry when set, the
// previous arc's terminal value otherwise. On the first arc there is no previous
// value, so the frame's natural value at the vertical-hold exit is used — the
// rocket is still flying straight up there, so ϑ = 90° and α = 0 describe the
// same attitude.
func entryOf(s PitchSegment, first bool, prev float64) float64 {
	if s.Entry != nil {
		return *s.Entry
	}
	if first {
		if s.Frame == FrameAlpha {
			return 0
		}
		return math.Pi / 2
	}
	return prev
}

// segAt locates the arc covering t, returning it together with its entry value
// a, its entry rate aRate and start time t0. ok is false during the vertical
// hold, past the last segment, and when the program has no segments at all.
//
// aRate is only meaningful for ShapeHermite, which chains it from the previous
// arc's terminal rate to stay C¹. The first arc enters at rate 0, matching the
// vertical hold it leaves.
func (p PitchProgram) segAt(t float64) (seg PitchSegment, a, aRate, t0 float64, ok bool) {
	if t <= p.TVert {
		return PitchSegment{}, 0, 0, 0, false
	}
	prev := math.Pi / 2
	prevRate := 0.0
	t0 = p.TVert
	for i, s := range p.Segments {
		a = entryOf(s, i == 0, prev)
		if t <= s.TEnd {
			return s, a, prevRate, t0, true
		}
		prev, prevRate, t0 = s.Val, s.K, s.TEnd
	}
	return PitchSegment{}, 0, 0, 0, false
}

// Cmd returns the commanded body pitch ϑ_пр [rad] at time t, given the current
// flight-path angle theta [rad]. theta is used only by FrameAlpha arcs, for
// which ϑ = θ + α_пр(t); FrameAlpha therefore makes the program a function of
// state, not of time alone. Before the program it holds 90° (the vertical hold);
// past it, the final segment's terminal value in that segment's frame.
func (p PitchProgram) Cmd(t, theta float64) float64 {
	seg, a, aRate, t0, ok := p.segAt(t)
	if !ok {
		if t > p.TVert && len(p.Segments) > 0 {
			last := p.Segments[len(p.Segments)-1]
			return referenced(last.Frame, last.Val, theta)
		}
		return math.Pi / 2
	}
	v := arcAt(seg.Shape, a, aRate, seg.Val, t0, seg.TEnd, seg.K, t)
	return referenced(seg.Frame, v, theta)
}

// Rate returns the programmed pitch rate ϑ̇_пр(t) [rad/s]: the analytical
// derivative of the active arc, plus the flight-path rate thetaDot for
// FrameAlpha arcs (ϑ̇ = θ̇ + α̇_пр). It is 0 during the vertical hold and past
// the program; at cosine joints it reduces to θ̇ under FrameAlpha and to 0 under
// FrameTheta.
func (p PitchProgram) Rate(t, thetaDot float64) float64 {
	seg, a, aRate, t0, ok := p.segAt(t)
	if !ok {
		return 0
	}
	r := arcRate(seg.Shape, a, aRate, seg.Val, t0, seg.TEnd, seg.K, t)
	return referenced(seg.Frame, r, thetaDot)
}

// referenced adds the flight-path quantity (θ or θ̇) to an α-framed arc value,
// and passes a ϑ-framed value through untouched.
func referenced(f Frame, v, flightPath float64) float64 {
	if f == FrameAlpha {
		return flightPath + v
	}
	return v
}

// Validate checks that the phase times are strictly increasing (TVert < TEnd_0 <
// TEnd_1 < …), which the arc interpolation relies on. Entry values at steering
// frame changes are not checked here: Simulate latches them from the state.
func (p PitchProgram) Validate() error {
	if p.TVert < 0 {
		return fmt.Errorf("pitch: t_vertical must be ≥ 0, got %g", p.TVert)
	}
	prev := p.TVert
	for i, seg := range p.Segments {
		if seg.TEnd <= prev {
			return fmt.Errorf("pitch: segment %d end time %g must exceed previous %g", i, seg.TEnd, prev)
		}
		prev = seg.TEnd
	}
	return nil
}

// arcAt evaluates the arc of the given shape from a (at t0) to b (at t1) at t.
// aRate is the arc's entry rate and is used only by ShapeHermite.
func arcAt(sh Shape, a, aRate, b, t0, t1, k, t float64) float64 {
	switch sh {
	case ShapeCos:
		return arcCos(a, b, t0, t1, k, t)
	case ShapeHermite:
		return arcHermite(a, aRate, b, k, t0, t1, t)
	default:
		return arcExp(a, b, t0, t1, k, t)
	}
}

// arcHermite is the unique cubic on [t0,t1] matching both endpoint values and
// both endpoint rates — the C¹ arc:
//
//	ϑ = (2s³−3s²+1)·a + (s³−2s²+s)·h·ȧ + (−2s³+3s²)·b + (s³−s²)·h·ḃ,
//	s = (t−t0)/h,  h = t1−t0
//
// ȧ is the previous arc's terminal rate (0 for the first arc, matching the
// vertical hold) and ḃ is this arc's own K. Chaining ȧ from the predecessor is
// what makes a whole program C¹ without any extra parameters: each arc still
// carries exactly one free number, as exp and cos do.
func arcHermite(a, aRate, b, bRate, t0, t1, t float64) float64 {
	h := t1 - t0
	s := (t - t0) / h
	s2 := s * s
	s3 := s2 * s
	return (2*s3-3*s2+1)*a + (s3-2*s2+s)*h*aRate +
		(-2*s3+3*s2)*b + (s3-s2)*h*bRate
}

// arcHermiteRate is the analytical derivative of arcHermite [rad/s]. Unlike the
// exp and cos rates it is deliberately NOT forced to zero at the joints: it
// returns aRate at s=0 and bRate at s=1, which is the whole point of the shape.
func arcHermiteRate(a, aRate, b, bRate, t0, t1, t float64) float64 {
	h := t1 - t0
	s := (t - t0) / h
	s2 := s * s
	return ((6*s2-6*s)*a + (3*s2-4*s+1)*h*aRate +
		(-6*s2+6*s)*b + (3*s2-2*s)*h*bRate) / h
}

// arcExp is the smoothed exponential segment from a (at t0) to b (at t1):
//
//	ϑ = a + (b−a) · (e^(−k·u) − 1)/(e^(−k) − 1),
//	u = 6s^5 − 15s^4 + 10s^3,  s = (t−t0)/(t1−t0)
//
// k>0 front-loads the turn, keeping the pitch rate — and so |α| — low in the
// later, supersonic part of each arc. k<0 back-loads it. k→0 reduces to the
// smootherstep curve. The smootherstep phase makes stacked exponential arcs
// value-, rate-, and acceleration-continuous at joints.
func arcExp(a, b, t0, t1, k, t float64) float64 {
	s := (t - t0) / (t1 - t0)
	u := smootherstep(s)
	return a + (b-a)*expEase(u, k)
}

// arcCos is the chained half-cosine segment from a (at t0) to b (at t1):
//
//	ϑ = (a+b)/2 + (a−b)/2 · cos(π · s^k),  s = (t−t0)/(t1−t0)
//
// This is the local-segment form of the C++ program formula, with a equal to the
// previous segment's terminal angle (or 90° for the first segment). It reaches a
// at t0 and b at t1. k≥1 starts and ends the turn smoothly (ϑ̇=0 at the joints).
// The C++ reference uses k=1.1.
func arcCos(a, b, t0, t1, k, t float64) float64 {
	k = smoothCosK(k)
	s := (t - t0) / (t1 - t0)
	return (a+b)/2 + (a-b)/2*math.Cos(math.Pi*math.Pow(s, k))
}

func smoothCosK(k float64) float64 {
	if k < minKCos {
		return minKCos
	}
	return k
}

func smootherstep(s float64) float64 {
	return s * s * s * (s*(s*6-15) + 10)
}

func smootherstepPrime(s float64) float64 {
	return 30 * s * s * (s - 1) * (s - 1)
}

func expEase(u, k float64) float64 {
	if math.Abs(k) < 1e-9 {
		return u
	}
	return math.Expm1(-k*u) / math.Expm1(-k)
}

func expEasePrime(u, k float64) float64 {
	if math.Abs(k) < 1e-9 {
		return 1
	}
	return -k * math.Exp(-k*u) / math.Expm1(-k)
}

// arcRate is the analytical time derivative of the corresponding arc [rad/s].
// aRate is the arc's entry rate and is used only by ShapeHermite.
func arcRate(sh Shape, a, aRate, b, t0, t1, k, t float64) float64 {
	tau := t1 - t0
	s := (t - t0) / tau
	if sh == ShapeHermite {
		// No joint clamp here on purpose: a Hermite arc's rate at its endpoints
		// is exactly the chained entry/terminal rate, which is what keeps the
		// program C¹.
		return arcHermiteRate(a, aRate, b, k, t0, t1, t)
	}
	if sh == ShapeCos {
		if s <= 0 || s >= 1 {
			return 0 // defined exactly at joints to avoid roundoff in diagnostics
		}
		k = smoothCosK(k)
		return -(a - b) / 2 * math.Sin(math.Pi*math.Pow(s, k)) * math.Pi * k * math.Pow(s, k-1) / tau
	}
	if s <= 0 || s >= 1 {
		return 0
	}
	u := smootherstep(s)
	return (b - a) * expEasePrime(u, k) * smootherstepPrime(s) / tau
}
