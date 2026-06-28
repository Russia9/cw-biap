package traj

import (
	"math"

	na "github.com/Russia9/numerical-analysis"
	"traj/atmosphere"
)

// Row is one sampled trajectory point with derived quantities for output.
type Row struct {
	T     float64 // time [s]
	M     float64 // mass [kg]
	Vx    float64 // [m/s]
	Vy    float64 // [m/s]
	Xpos  float64 // launch-frame x [m]
	Ypos  float64 // launch-frame y [m]
	H     float64 // altitude [m]
	V     float64 // speed [m/s]
	Pitch float64 // ϑ [rad]
	Theta float64 // flight path angle θ [rad]
	Alpha float64 // angle of attack α = ϑ−θ [rad]
	Mach  float64
	Q     float64 // dynamic pressure [Pa]
	Drag  float64 // X [N]
	Lift  float64 // Y [N]
	Mz    float64 // pitch moment [N·m]
	Omega float64 // pitch rate ϑ̇ / ω_z [rad/s]
	Stage int     // 1..3 active, 4 = payload (passive)
}

// Diagnostics collects the constructive-ballistic check values (§4.4) and the
// terminal trajectory parameters.
type Diagnostics struct {
	MaxQ, MaxQt            float64 // max dynamic pressure [Pa] and its time [s]
	MaxAlphaSub            float64 // max |α| for M ≤ 1.1 [deg]
	MaxAlphaSup            float64 // max |α| for M > 1.1 and H ≤ Hatm [deg]
	MaxPitchRate           float64 // max |ϑ̇| on the active leg [deg/s]
	PitchRateSep1          float64 // |ϑ̇| at stage-1 separation [deg/s]
	PitchRateSep2          float64 // |ϑ̇| at stage-2 separation [deg/s]
	CrossUpTime, CrossUpH  float64 // 94 km crossing, ascending
	CrossUpStage           int
	CrossDownTime          float64 // 94 km crossing, descending
	BurnoutT, BurnoutV     float64 // at stage-3 burnout (Tk3)
	BurnoutH, BurnoutTheta float64
	ApogeeT, ApogeeH       float64
	ImpactT, ImpactRange   float64
}

// Simulate integrates the full flight: three powered stages (5-state, programmed
// pitch) followed by the passive payload (7-state, free pitch under the
// aerodynamic moment), then derives the constraint diagnostics. The pitch-program
// free parameters live in r.Pitch so an optimizer can vary them via r.
func Simulate(r Rocket, at *AeroTable, h float64) ([]Row, Diagnostics, error) {
	var rows []Row

	state := []float64{0, 0, 0, 0, r.Stages[0].M0}
	t0 := 0.0
	tk := []float64{r.Pitch.Tk1, r.Pitch.Tk2, r.Pitch.Tk3}

	for i, st := range r.Stages {
		tEnd := tk[i]
		sys := r.activeSystem(st, at)
		res, err := na.RungeKuttaMethod(sys, t0, state, []float64{tEnd}, h, stopAtTime(tEnd))
		if err != nil {
			return nil, Diagnostics{}, err
		}
		rows = appendRows(rows, r, at, res, st.Part, i+1, true, i > 0)
		state = finalState(res, 5)
		if i < len(r.Stages)-1 {
			state[iM] = r.Stages[i+1].M0 // drop spent stage, expose next sub-rocket
		} else {
			state[iM] = r.Payload // drop stage 3, payload remains
		}
		t0 = tEnd
	}

	// Active -> passive: same 5 states. The payload flies velocity-aligned
	// (drag-only), so no pitch state is carried.
	ps := []float64{state[iVx], state[iVy], state[iX], state[iY], state[iM]}
	tMax := t0 + 6000
	psys := r.passiveSystem(at)
	res, err := na.RungeKuttaMethod(psys, t0, ps, []float64{}, h, stopGround(tMax))
	if err != nil {
		return nil, Diagnostics{}, err
	}
	rows = appendRows(rows, r, at, res, PayloadPart, 4, false, true)

	return rows, diagnose(r, rows), nil
}

func stopAtTime(tEnd float64) func(x float64, y ...float64) (bool, bool) {
	return func(x float64, _ ...float64) (bool, bool) {
		done := x >= tEnd-1e-9
		return done, done
	}
}

func stopGround(tMax float64) func(x float64, y ...float64) (bool, bool) {
	return func(x float64, y ...float64) (bool, bool) {
		H := Altitude(y)
		if x >= tMax {
			return true, true
		}
		return H <= 0, H <= 0 && math.Abs(H) < 1.0
	}
}

// finalState extracts the last state vector (dimension d) from an RK result.
func finalState(res [][]na.Point2D, d int) []float64 {
	last := len(res[0]) - 1
	s := make([]float64, d)
	for i := 0; i < d; i++ {
		s[i] = res[i][last].Y
	}
	return s
}

// appendRows converts an RK result into trajectory rows. skipFirst drops the
// leading step to avoid duplicating the time shared with the previous segment.
func appendRows(rows []Row, r Rocket, at *AeroTable, res [][]na.Point2D, part string, stage int, active, skipFirst bool) []Row {
	d := len(res)
	n := len(res[0])
	y := make([]float64, d)
	start := 0
	if skipFirst {
		start = 1
	}
	for k := start; k < n; k++ {
		t := res[0][k].X
		for i := 0; i < d; i++ {
			y[i] = res[i][k].Y
		}
		rows = append(rows, rowFrom(r, at, t, y, part, stage, active))
	}
	return rows
}

func rowFrom(r Rocket, at *AeroTable, t float64, y []float64, part string, stage int, active bool) Row {
	H := Altitude(y)
	V := VelMag(y)
	theta := FlightAngle(y)
	_, rho, _, _, _, a := atmosphere.Atmosphere(H)
	mach := 0.0
	if a > 0 {
		mach = V / a
	}
	q := 0.5 * rho * V * V

	var pit, om float64
	if active {
		pit = r.Pitch.Theta(t)
		om = r.Pitch.Rate(t)
	} else {
		pit = FlightAngle(y) // velocity-aligned payload ⇒ α=0
		om = 0
	}
	X, Y, Mz := AeroForces(at, part, pit, y)
	return Row{
		T: t, M: y[iM], Vx: y[iVx], Vy: y[iVy], Xpos: y[iX], Ypos: y[iY],
		H: H, V: V, Pitch: pit, Theta: theta, Alpha: pit - theta,
		Mach: mach, Q: q, Drag: X, Lift: Y, Mz: Mz, Omega: om, Stage: stage,
	}
}

// surfaceRange returns the downrange distance along the surface [m].
func surfaceRange(x, y float64) float64 {
	return Rz * math.Atan2(x, Rz+y)
}

func diagnose(r Rocket, rows []Row) Diagnostics {
	const r2d = 180 / math.Pi
	d := Diagnostics{}
	d.PitchRateSep1 = math.Abs(r.Pitch.Rate(r.Pitch.Tk1)) * r2d
	d.PitchRateSep2 = math.Abs(r.Pitch.Rate(r.Pitch.Tk2)) * r2d

	crossedUp := false
	prevH := 0.0
	for i, row := range rows {
		if row.Stage <= 3 { // active leg — the §4.4 checks apply here
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

	// Burnout = last active row (stage 3, at Tk3).
	for i := len(rows) - 1; i >= 0; i-- {
		if rows[i].Stage == 3 {
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
