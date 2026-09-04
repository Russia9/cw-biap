package traj

type Pitch []PitchArc

/*
 * PitchArc defines a curve for the pitch(t).
 * Each pitchArc is active from either the start or the TEnd of the previous one.
 * Each pitchArc moves theta from ThetaDeg of previous one to ThetaDeg of current one.
 */
type PitchArc struct {
	TEnd     float64 `json:"t_end"`
	Shape    string  `json:"shape"`
	K        float64 `json:"k"`
	ThetaDeg float64 `json:"theta_deg"`
}

func (p Pitch) Pitch(t float64) float64 {
	// TODO
	return 0
}
