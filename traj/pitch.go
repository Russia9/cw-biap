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

type pitchArc func(tStart, tEnd, thetaDegStart, thetaDegEnd, k, t float64) float64

var pitchShapes = map[string]pitchArc{
	"cos": func(tStart, tEnd, thetaDegStart, thetaDegEnd, k, t float64) float64 {
		return (thetaDegStart+thetaDegEnd)/2 + (thetaDegStart-thetaDegEnd)/2*math.Cos(math.Pi*math.Pow((t-tStart)/(tEnd-tStart), k))
	},
}

func (p Pitch) segment(t float64) int {
	if t <= p.TStart {
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

func (p Pitch) Pitch(t float64) float64 {
	is := p.segment(t)
	if is == -1 {
		return p.ThetaDegStart
	}
	s := p.Segments[is]

	thetaDegStart := p.ThetaDegStart
	tStart := p.TStart
	if is > 0 {
		thetaDegStart = p.Segments[is-1].ThetaDeg
		tStart = p.Segments[is-1].TEnd
	}

	return pitchShapes[s.Shape](tStart, s.TEnd, thetaDegStart, s.ThetaDeg, s.K, t)
}
