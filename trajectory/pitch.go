package traj

import (
	"math"
	"slices"
)

type Pitch struct {
	ThetaDegStart float64        `json:"theta_deg_start"` // theta_deg at t=0
	TStart        float64        `json:"t_start"`         // time when first segment starts
	Segments      []PitchSegment `json:"segments"`
}

/*
 * PitchSegment defines a curve for the pitch(t).
 * Each PitchSegment is active from either the start or the TEnd of the previous one.
 * Each PitchSegment moves theta from ThetaDeg of previous one to ThetaDeg of current one.
 */
type PitchSegment struct {
	TEnd     float64 `json:"t_end"`
	Shape    string  `json:"shape"`
	K        float64 `json:"k"`
	ThetaDeg float64 `json:"theta_deg"`
}

// exitSlope is what a segment hands to the one after it, deg/s.
func exitSlope(s PitchSegment) float64 {
	if s.Shape == PitchShapeHermite {
		return s.K
	}
	return 0
}

type pitchArc func(tStart, tEnd, thetaDegStart, thetaDegEnd, slopeStart, k, t float64) float64

const PitchShapeCos = "cos"
const PitchShapeHermite = "hermite"

var pitchShapes = map[string]pitchArc{
	PitchShapeCos: func(tStart, tEnd, thetaDegStart, thetaDegEnd, _, k, t float64) float64 {
		return (thetaDegStart+thetaDegEnd)/2 + (thetaDegStart-thetaDegEnd)/2*math.Cos(math.Pi*math.Pow((t-tStart)/(tEnd-tStart), k))
	},
	PitchShapeHermite: func(tStart, tEnd, thetaDegStart, thetaDegEnd, slopeStart, k, t float64) float64 {
		h := tEnd - tStart
		if h <= 0 {
			return thetaDegEnd
		}
		s := t - tStart
		switch {
		case s <= 0:
			return thetaDegStart
		case s >= h:
			return thetaDegEnd
		}

		delta := (thetaDegEnd - thetaDegStart) / h // secant rate, deg/s

		m0, m1 := slopeStart, k
		a := thetaDegStart
		b := m0
		c := (3*delta - 2*m0 - m1) / h
		d := (m0 + m1 - 2*delta) / (h * h)

		return a + s*(b+s*(c+s*d)) // Horner
	},
}

// segment returns the index of the arc active at t, -1 before the program
// starts, or len(Segments) once t is past the last arc.
func (p Pitch) segment(t float64) int {
	if t <= p.TStart || len(p.Segments) == 0 {
		return -1
	}
	i, _ := slices.BinarySearchFunc(p.Segments, t, func(s PitchSegment, t float64) int {
		if s.TEnd < t {
			return -1
		} else {
			return 1
		}
	})
	return i
}

// Pitch returns the current programmed pitch in radians.
//
// Past the last arc the final commanded angle is HELD. Evaluating the arc
// beyond its own TEnd would put (t-tStart)/(tEnd-tStart) above 1, and cos(pi*x^k)
// then swings the vehicle back and forth for the whole coast; holding is what an
// open-loop program actually does once it runs out of instructions.
func (p Pitch) Pitch(t float64) float64 {
	is := p.segment(t)
	if is == -1 {
		return p.ThetaDegStart * d2r
	}
	if is >= len(p.Segments) {
		return p.Segments[len(p.Segments)-1].ThetaDeg * d2r
	}
	s := p.Segments[is]

	thetaDegStart := p.ThetaDegStart
	tStart := p.TStart
	slopeStart := 0.0
	if is > 0 {
		prev := p.Segments[is-1]
		thetaDegStart = prev.ThetaDeg
		tStart = prev.TEnd
		slopeStart = exitSlope(prev)
	}

	return pitchShapes[s.Shape](tStart, s.TEnd, thetaDegStart, s.ThetaDeg, slopeStart, s.K, t) * d2r
}
