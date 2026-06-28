package traj

import "math"

// pitchExp is the exponent of the cosine pitch law (§4.3).
const pitchExp = 1.1

// arc is one smooth cosine segment from angle a (at t0) to angle b (at t1):
//
//	ϑ = (a+b)/2 + (a−b)/2 · cos(π·((t−t0)/(t1−t0))^1.1)
//
// It reaches exactly a at t0 and b at t1, with zero slope at both ends.
func arc(a, b, t0, t1, t float64) float64 {
	frac := (t - t0) / (t1 - t0)
	return (a+b)/2 + (a-b)/2*math.Cos(math.Pi*math.Pow(frac, pitchExp))
}

// Theta returns the programmed pitch angle ϑ_пр(t) [rad] on the active leg.
// Four phases: vertical hold, then three cosine arcs to ϑ_k1, ϑ_k2, ϑ_k3.
// For t beyond Tk3 it holds ϑ_k3 (the program is only used while powered).
func (p PitchParams) Theta(t float64) float64 {
	switch {
	case t <= p.Tv:
		return math.Pi / 2
	case t <= p.Tk1:
		return arc(math.Pi/2, p.ThK1, p.Tv, p.Tk1, t)
	case t <= p.Tk2:
		return arc(p.ThK1, p.ThK2, p.Tk1, p.Tk2, t)
	case t <= p.Tk3:
		return arc(p.ThK2, p.ThK3, p.Tk2, p.Tk3, t)
	default:
		return p.ThK3
	}
}

// Rate returns the programmed pitch rate ϑ̇_пр(t) [rad/s] via central difference
// (the cosine law has no convenient closed form at the phase joints).
func (p PitchParams) Rate(t float64) float64 {
	const h = 1e-4
	return (p.Theta(t+h) - p.Theta(t-h)) / (2 * h)
}
