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
	// ShapeCos is the half-cosine arc ported from the C++ reference
	// (traj-example/model.hpp).
	ShapeCos
)

// Default shape exponents when a config arc leaves k unset.
const (
	defaultKExp = 3.0 // exp: >0 front-loads the turn, 0 ⇒ linear
	defaultKCos = 1.1 // cos: power on normalized time (C++ reference value)
)

// PitchSegment is one arc of the pitch program. It interpolates from its entry
// angle (the previous segment's terminal angle, or 90° for the first, just after
// the vertical hold) to Theta, reaching Theta exactly at TEnd.
type PitchSegment struct {
	TEnd  float64 // absolute segment end time [s]
	Theta float64 // terminal pitch angle ϑ [rad]
	Shape Shape
	K     float64 // shape exponent (see defaultKExp / defaultKCos)
}

// PitchProgram is the programmed pitch ϑ_пр(t) [rad] on the active leg: a
// vertical hold of TVert seconds at 90°, then the chained arcs in Segments
// (ascending TEnd). Past the last segment it holds the final angle (the program
// is only used while powered).
type PitchProgram struct {
	TVert    float64
	Segments []PitchSegment
}

// Theta returns the programmed pitch angle ϑ_пр(t) [rad].
func (p PitchProgram) Theta(t float64) float64 {
	if t <= p.TVert || len(p.Segments) == 0 {
		return math.Pi / 2
	}
	a, t0 := math.Pi/2, p.TVert
	for _, seg := range p.Segments {
		if t <= seg.TEnd {
			return arcAt(seg.Shape, a, seg.Theta, t0, seg.TEnd, seg.K, t)
		}
		a, t0 = seg.Theta, seg.TEnd
	}
	return p.Segments[len(p.Segments)-1].Theta
}

// Rate returns the programmed pitch rate ϑ̇_пр(t) [rad/s] as the analytical
// derivative of the active arc (0 during the vertical hold and past the program).
func (p PitchProgram) Rate(t float64) float64 {
	if t <= p.TVert || len(p.Segments) == 0 {
		return 0
	}
	a, t0 := math.Pi/2, p.TVert
	for _, seg := range p.Segments {
		if t <= seg.TEnd {
			return arcRate(seg.Shape, a, seg.Theta, t0, seg.TEnd, seg.K, t)
		}
		a, t0 = seg.Theta, seg.TEnd
	}
	return 0
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

// arcExp is the exponential segment from a (at t0) to b (at t1):
//
//	ϑ = b + (a−b) · (e^(−k·s) − e^(−k))/(1 − e^(−k)),  s = (t−t0)/(t1−t0)
//
// k>0 front-loads the turn (steep at t0, flattening toward t1), keeping the pitch
// rate — and so |α| — low in the later, supersonic part of each arc. k→0 reduces
// to a straight line.
func arcExp(a, b, t0, t1, k, t float64) float64 {
	s := (t - t0) / (t1 - t0)
	if math.Abs(k) < 1e-9 {
		return a + (b-a)*s // linear limit
	}
	w := (math.Exp(-k*s) - math.Exp(-k)) / (1 - math.Exp(-k))
	return b + (a-b)*w
}

// arcCos is the half-cosine segment from a (at t0) to b (at t1):
//
//	ϑ = (a+b)/2 + (a−b)/2 · cos(π · s^k),  s = (t−t0)/(t1−t0)
//
// It reaches a at t0 and b at t1. k≥1 starts the turn smoothly (ϑ̇=0 at t0). The
// C++ reference uses k=1.1.
func arcCos(a, b, t0, t1, k, t float64) float64 {
	s := (t - t0) / (t1 - t0)
	return (a+b)/2 + (a-b)/2*math.Cos(math.Pi*math.Pow(s, k))
}

// arcRate is the analytical time derivative of the corresponding arc [rad/s].
func arcRate(sh Shape, a, b, t0, t1, k, t float64) float64 {
	tau := t1 - t0
	s := (t - t0) / tau
	if sh == ShapeCos {
		if s <= 0 {
			return 0 // s^(k-1) → 0 for k≥1; defined to 0 at the joint
		}
		return -(a - b) / 2 * math.Sin(math.Pi*math.Pow(s, k)) * math.Pi * k * math.Pow(s, k-1) / tau
	}
	if math.Abs(k) < 1e-9 {
		return (b - a) / tau // linear limit
	}
	return (a - b) * (-k * math.Exp(-k*s) / (1 - math.Exp(-k))) / tau
}
