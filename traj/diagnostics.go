package traj

import "math"

// Diagnostics collects the constructive-ballistic check values (§4.4) and the
// terminal trajectory parameters.
type Diagnostics struct {
	MaxQ, MaxQt           float64 // max dynamic pressure [Pa] and its time [s]
	MaxAlphaSub           float64 // max |α| for M ≤ 1.1 [deg]
	MaxAlphaSup           float64 // max |α| for M > 1.1 and H ≤ Hatm [deg]
	MaxPitchRate          float64 // max |ϑ̇| on the active leg [deg/s]
	MaxPitchRateNum       float64 // max |Δϑ/Δt| between active rows [deg/s]
	PitchRateSep1         float64 // |ϑ̇| at stage-1 separation [deg/s]
	PitchRateSep2         float64 // |ϑ̇| at stage-2 separation [deg/s]
	AlphaSep1, AlphaSep2  float64 // |α| at stage-1 / stage-2 separation [deg]
	QSep1, QSep2          float64 // q at those separations [Pa]
	CrossUpTime, CrossUpH float64 // 94 km crossing, ascending
	CrossUpStage          int
	// CrossUpMargin is how far inside the stage-2 burn the ascending crossing
	// sits [s]; negative means outside it. §4.1 is stated in terms of the stage
	// number, but a stage number is a step function of the design vector and
	// gives an optimizer no direction to move once it is on the wrong side —
	// this is the same predicate with a usable slope.
	CrossUpMargin          float64
	CrossDownTime          float64 // 94 km crossing, descending
	BurnoutT, BurnoutV     float64 // at stage-3 burnout (Tk3)
	BurnoutH, BurnoutTheta float64
	ApogeeT, ApogeeH       float64
	ImpactT, ImpactRange   float64
	GroundHitStage         int // >0 if that powered stage reached the ground
}

// AlphaSepMax returns the largest |α| [deg] over the stage separations that
// happen inside the atmosphere (q > QSepMin), or 0 when none qualifies — which
// reads as "no violation" through the optimizer's penalty, the right default
// for a constraint that simply does not apply to this trajectory.
//
// The gate lives here rather than at measurement time so that the printed
// diagnostics can still report the raw α and q of an exempt separation and say
// why it was exempt.
func (d Diagnostics) AlphaSepMax() float64 {
	worst := 0.0
	for _, s := range [...]struct{ alpha, q float64 }{
		{d.AlphaSep1, d.QSep1},
		{d.AlphaSep2, d.QSep2},
	} {
		if s.q > QSepMin {
			worst = math.Max(worst, s.alpha)
		}
	}
	return worst
}

// surfaceRange returns the downrange distance along the surface [m].
func surfaceRange(x, y float64) float64 {
	return Rz * math.Atan2(x, Rz+y)
}

func diagnose(r Rocket, rows []Row, tk []float64) Diagnostics {
	d := Diagnostics{}
	lastActive := len(r.Stages) // stage index of the final powered stage
	sepOmega := make([]float64, lastActive)
	sepAlpha := make([]float64, lastActive)
	sepQ := make([]float64, lastActive)
	crossedUp := false
	prevH := 0.0
	// Previous active row, for the finite-difference pitch rate.
	prevT, prevPitch := 0.0, 0.0
	havePrev := false
	// Audit note (2026-08): the §4.4 maxima are gated to the active leg on
	// purpose. The α/ϑ̇ limits only exist for controlled flight (the passive
	// warhead flies velocity-aligned, α≡0 by model), and the q limit is the
	// ascent structural limit — re-entry dynamic pressure on the warhead is a
	// different regime §4.4 does not govern. α itself is never angle-wrapped:
	// valid while the planar trajectory keeps Vx > 0, which every shipped
	// config does; a config that turns past the vertical would need wrapping
	// here and in AeroForces.
	for i, row := range rows {
		if row.Stage <= lastActive { // active leg — the §4.4 checks apply here
			// Last active row of a stage sits exactly at its burnout time.
			sepOmega[row.Stage-1] = row.Omega
			sepAlpha[row.Stage-1] = row.Alpha
			sepQ[row.Stage-1] = row.Q
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
	// single-stage config has neither. The last powered stage's burnout is
	// payload release, not a staging event, so it is deliberately not read
	// here — and it is exempt from the |α| gate anyway (q ≈ 0 up there, and α
	// is identically zero on the passive leg).
	if len(tk) > 1 {
		d.PitchRateSep1 = math.Abs(sepOmega[0]) * r2d
		d.AlphaSep1, d.QSep1 = math.Abs(sepAlpha[0])*r2d, sepQ[0]
	}
	if len(tk) > 2 {
		d.PitchRateSep2 = math.Abs(sepOmega[1]) * r2d
		d.AlphaSep2, d.QSep2 = math.Abs(sepAlpha[1])*r2d, sepQ[1]
	}

	// §4.1 expects the ascending 94 km crossing inside the stage-2 burn. A
	// config with no stage 2 has no such expectation, hence the same guard the
	// separation diagnostics use.
	if len(tk) > 1 {
		lo, hi := tk[0], tk[1]
		if crossedUp {
			d.CrossUpMargin = math.Min(d.CrossUpTime-lo, hi-d.CrossUpTime)
		} else {
			// Never reached 94 km: a bounded sentinel one window-width outside,
			// rather than an infinity. Such a flight is already dominated by
			// its range term and does not need an unbounded penalty on top.
			d.CrossUpMargin = lo - hi
		}
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
