package traj

import "math"

// Diagnostics collects the constructive-ballistic check values (§4.4) and the
// terminal trajectory parameters.
type Diagnostics struct {
	MaxQ, MaxQt            float64 // max dynamic pressure [Pa] and its time [s]
	MaxAlphaSub            float64 // max |α| for M ≤ 1.1 [deg]
	MaxAlphaSup            float64 // max |α| for M > 1.1 and H ≤ Hatm [deg]
	MaxPitchRate           float64 // max |ϑ̇| on the active leg [deg/s]
	MaxPitchRateNum        float64 // max |Δϑ/Δt| between active rows [deg/s]
	PitchRateSep1          float64 // |ϑ̇| at stage-1 separation [deg/s]
	PitchRateSep2          float64 // |ϑ̇| at stage-2 separation [deg/s]
	CrossUpTime, CrossUpH  float64 // 94 km crossing, ascending
	CrossUpStage           int
	CrossDownTime          float64 // 94 km crossing, descending
	BurnoutT, BurnoutV     float64 // at stage-3 burnout (Tk3)
	BurnoutH, BurnoutTheta float64
	ApogeeT, ApogeeH       float64
	ImpactT, ImpactRange   float64
	GroundHitStage         int // >0 if that powered stage reached the ground
}

// surfaceRange returns the downrange distance along the surface [m].
func surfaceRange(x, y float64) float64 {
	return Rz * math.Atan2(x, Rz+y)
}

func diagnose(r Rocket, rows []Row, tk []float64) Diagnostics {
	d := Diagnostics{}
	lastActive := len(r.Stages) // stage index of the final powered stage
	sepOmega := make([]float64, lastActive)
	crossedUp := false
	prevH := 0.0
	// Previous active row, for the finite-difference pitch rate.
	prevT, prevPitch := 0.0, 0.0
	havePrev := false
	for i, row := range rows {
		if row.Stage <= lastActive { // active leg — the §4.4 checks apply here
			// Last active row of a stage sits exactly at its burnout time.
			sepOmega[row.Stage-1] = row.Omega
			if havePrev && row.T > prevT {
				rate := math.Abs(row.Pitch-prevPitch) / (row.T - prevT) * r2d
				d.MaxPitchRateNum = math.Max(d.MaxPitchRateNum, rate)
			}
			prevT, prevPitch, havePrev = row.T, row.Pitch, true
			if row.Q > d.MaxQ {
				d.MaxQ, d.MaxQt = row.Q, row.T
			}
			if row.Q > 1.0 { // α only meaningful once there is airflow
				aDeg := math.Abs(row.Alpha) * r2d
				if row.Mach <= 1.1 {
					d.MaxAlphaSub = math.Max(d.MaxAlphaSub, aDeg)
				} else if row.H <= Hatm {
					d.MaxAlphaSup = math.Max(d.MaxAlphaSup, aDeg)
				}
			}
			d.MaxPitchRate = math.Max(d.MaxPitchRate, math.Abs(row.Omega)*r2d)
		}
		if i > 0 {
			if !crossedUp && prevH < Hatm && row.H >= Hatm {
				d.CrossUpTime, d.CrossUpH, d.CrossUpStage = row.T, row.H, row.Stage
				crossedUp = true
			}
			if crossedUp && d.CrossDownTime == 0 && prevH >= Hatm && row.H < Hatm {
				d.CrossDownTime = row.T
			}
		}
		if row.H > d.ApogeeH {
			d.ApogeeH, d.ApogeeT = row.H, row.T
		}
		prevH = row.H
	}

	// Separation rates are only defined where a staging event exists; a
	// single-stage config has neither.
	if len(tk) > 1 {
		d.PitchRateSep1 = math.Abs(sepOmega[0]) * r2d
	}
	if len(tk) > 2 {
		d.PitchRateSep2 = math.Abs(sepOmega[1]) * r2d
	}

	// Burnout = last active row (final stage, at its burnout time).
	for i := len(rows) - 1; i >= 0; i-- {
		if rows[i].Stage == lastActive {
			d.BurnoutT = rows[i].T
			d.BurnoutV = rows[i].V
			d.BurnoutH = rows[i].H
			d.BurnoutTheta = rows[i].Theta * r2d
			break
		}
	}

	last := rows[len(rows)-1]
	d.ImpactT = last.T
	d.ImpactRange = surfaceRange(last.Xpos, last.Ypos)
	return d
}
