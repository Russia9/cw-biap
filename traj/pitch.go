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
)

// Default shape exponents when a config arc leaves k unset.
const (
	defaultKExp = 3.0 // exp: >0 front-loads the turn, 0 ⇒ smootherstep-only
	defaultKCos = 1.1 // cos: power on normalized time (C++ reference value)
	minKCos     = 1.0 // lower values make visually sharp, non-smooth starts
)

// PitchSegment is one arc of the pitch program. It interpolates from its entry
// angle (the previous segment's terminal angle, or 90° for the first, just after
// the vertical hold) to Theta, reaching Theta exactly at TEnd.
type PitchSegment struct {
	TEnd  float64 // absolute segment end time [s]
	Theta float64 // terminal pitch angle ϑ [rad]
	Shape Shape
	K     float64 // shape exponent (cos values below minKCos are normalized)
}

// PitchProgram is the programmed pitch ϑ_пр(t) [rad] on the active leg: a
// vertical hold of TVert seconds at 90°, then the chained arcs in Segments
// (ascending TEnd). Past the last segment it holds the final angle (the program
// is only used while powered).
type PitchProgram struct {
	TVert    float64
	Segments []PitchSegment
}

// segAt locates the arc covering t, returning it together with its entry angle
// a and start time t0. ok is false during the vertical hold, past the last
// segment, and when the program has no segments at all.
func (p PitchProgram) segAt(t float64) (seg PitchSegment, a, t0 float64, ok bool) {
	if t <= p.TVert {
		return PitchSegment{}, 0, 0, false
	}
	a, t0 = math.Pi/2, p.TVert
	for _, s := range p.Segments {
		if t <= s.TEnd {
			return s, a, t0, true
		}
		a, t0 = s.Theta, s.TEnd
	}
	return PitchSegment{}, 0, 0, false
}

// Theta returns the programmed pitch angle ϑ_пр(t) [rad]. Before the program it
// holds 90° (the vertical hold); past it, the final segment's terminal angle.
func (p PitchProgram) Theta(t float64) float64 {
	seg, a, t0, ok := p.segAt(t)
	if !ok {
		if t > p.TVert && len(p.Segments) > 0 {
			return p.Segments[len(p.Segments)-1].Theta
		}
		return math.Pi / 2
	}
	return arcAt(seg.Shape, a, seg.Theta, t0, seg.TEnd, seg.K, t)
}

// Rate returns the programmed pitch rate ϑ̇_пр(t) [rad/s] as the analytical
// derivative of the active arc (0 during the vertical hold, at cosine joints,
// and past the program).
func (p PitchProgram) Rate(t float64) float64 {
	seg, a, t0, ok := p.segAt(t)
	if !ok {
		return 0
	}
	return arcRate(seg.Shape, a, seg.Theta, t0, seg.TEnd, seg.K, t)
}

// Validate checks that the phase times are strictly increasing (TVert < TEnd_0 <
// TEnd_1 < …), which the arc interpolation relies on.
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
func arcAt(sh Shape, a, b, t0, t1, k, t float64) float64 {
	if sh == ShapeCos {
		return arcCos(a, b, t0, t1, k, t)
	}
	return arcExp(a, b, t0, t1, k, t)
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
func arcRate(sh Shape, a, b, t0, t1, k, t float64) float64 {
	tau := t1 - t0
	s := (t - t0) / tau
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
