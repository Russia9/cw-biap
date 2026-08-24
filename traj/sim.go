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

// passiveMaxDuration caps the coast/re-entry leg [s] in case the payload never
// returns to the ground (numerical safety).
const passiveMaxDuration = 6000.0

// Simulate integrates the full flight: the powered stages (5-state, programmed
// pitch) followed by the passive payload (5-state, velocity-aligned drag-only),
// then derives the constraint diagnostics. The pitch-program free parameters live
// in r.Pitch so an optimizer can vary them via r.
func Simulate(r Rocket, at *AeroTable, h float64) ([]Row, Diagnostics, error) {
	var rows []Row
	state := []float64{0, 0, 0, 0, r.Stages[0].M0}
	t0 := 0.0
	tk := r.BurnoutTimes()

	// latchFrameEntry writes into the program, so work on a private copy.
	segs := make([]PitchSegment, len(r.Pitch.Segments))
	copy(segs, r.Pitch.Segments)
	r.Pitch.Segments = segs

	for i, st := range r.Stages {
		r.latchFrameEntry(t0, state)
		stageRows, end, err := r.simulateStage(at, st, i, t0, tk[i], state, h)
		if err != nil {
			return nil, Diagnostics{}, err
		}
		rows = append(rows, stageRows...)
		state = end
		if Altitude(state) < 0 {
			// Flew into the ground under power. The flight ends there: the
			// remaining stages never burn and the last row is the impact. This
			// scores as a very short range rather than an error, which keeps
			// the optimizer's objective continuous across the boundary.
			d := diagnose(r, rows, tk)
			d.GroundHitStage = i + 1
			return rows, d, nil
		}
		if i < len(r.Stages)-1 {
			state[iM] = r.Stages[i+1].M0 // drop spent stage, expose next sub-rocket
		} else {
			state[iM] = r.Payload // drop last stage, payload remains
		}
		t0 = tk[i]
	}

	passiveRows, err := r.simulatePassive(at, t0, state, h)
	if err != nil {
		return nil, Diagnostics{}, err
	}
	rows = append(rows, passiveRows...)

	return rows, diagnose(r, rows, tk), nil
}

// latchFrameEntry fixes the entry value of a steering-frame change occurring at
// time t, so the commanded ϑ is continuous across it.
//
// A ϑ arc following an α arc has to start from θ(t) + α, and an α arc following
// a ϑ arc from ϑ − θ(t); both depend on the flight-path angle, which is a state
// and so cannot be written into the config ahead of time. Steering is selected
// per stage, so a frame change only ever falls on a stage boundary, where
// Simulate holds the state. An explicit theta0_deg in the config wins.
func (r Rocket) latchFrameEntry(t float64, state []float64) {
	for i := 1; i < len(r.Pitch.Segments); i++ {
		s := &r.Pitch.Segments[i]
		if s.Entry != nil || s.Frame == r.Pitch.Segments[i-1].Frame {
			continue
		}
		if math.Abs(r.Pitch.Segments[i-1].TEnd-t) > 1e-9 {
			continue // this change belongs to a different stage boundary
		}
		theta := FlightAngle(state)
		v := r.Pitch.Cmd(t, theta) // the outgoing frame's ϑ at the joint
		if s.Frame == FrameAlpha {
			v -= theta
		}
		s.Entry = &v
	}
}

// simulateStage integrates one powered stage from t0 to tEnd and returns its
// trajectory rows and the final state vector (5 states).
func (r Rocket) simulateStage(at *AeroTable, st Stage, i int, t0, tEnd float64, state []float64, h float64) ([]Row, []float64, error) {
	sys := r.activeSystem(st, at)
	res, err := na.RungeKuttaMethod(sys, t0, state, []float64{tEnd}, h, stopAtTime(tEnd))
	if err != nil {
		return nil, nil, err
	}
	rows := appendRows(r, at, res, &st, st.AeroPart, i+1, i > 0)
	return rows, finalState(res), nil
}

// simulatePassive integrates the payload coast/re-entry from t0 to ground impact.
func (r Rocket) simulatePassive(at *AeroTable, t0 float64, state []float64, h float64) ([]Row, error) {
	// Active -> passive: same 5 states. The payload flies velocity-aligned
	// (drag-only), so no pitch state is carried.
	ps := []float64{state[iVx], state[iVy], state[iX], state[iY], state[iM]}
	res, err := na.RungeKuttaMethod(r.passiveSystem(at), t0, ps, []float64{}, h, stopGround(t0+passiveMaxDuration))
	if err != nil {
		return nil, err
	}
	return appendRows(r, at, res, nil, r.PayloadPart, 4, true), nil
}

// stopAtTime ends a powered stage at its burnout time, or earlier if the vehicle
// reaches the ground. Without the altitude guard a steering program that pitches
// past the horizon keeps integrating underground for the rest of the burn and
// hands the passive leg a sub-surface initial state, which its ground-stop
// bisection cannot resolve. H is exactly 0 at lift-off, so the test is strict.
func stopAtTime(tEnd float64) func(x float64, y ...float64) (bool, bool) {
	return func(x float64, y ...float64) (bool, bool) {
		done := x >= tEnd-1e-9 || Altitude(y) < 0
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

// finalState extracts the last state vector from an RK result.
func finalState(res [][]na.Point2D) []float64 {
	last := len(res[0]) - 1
	s := make([]float64, len(res))
	for i := range res {
		s[i] = res[i][last].Y
	}
	return s
}

// appendRows converts an RK result into trajectory rows. st is the powered
// stage the rows belong to, or nil on the passive leg. skipFirst drops the
// leading step to avoid duplicating the time shared with the previous segment.
func appendRows(r Rocket, at *AeroTable, res [][]na.Point2D, st *Stage, part string, stage int, skipFirst bool) []Row {
	var rows []Row
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
		rows = append(rows, rowFrom(r, at, t, y, st, part, stage))
	}
	return rows
}

func rowFrom(r Rocket, at *AeroTable, t float64, y []float64, st *Stage, part string, stage int) Row {
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
	if st != nil {
		// ϑ̇ needs θ̇, which needs the forces, which need ϑ — but ϑ never
		// depends on ϑ̇, so one forward evaluation closes the loop.
		ax, ay := r.activeAccel(*st, at, t, y)
		pit = r.Pitch.Cmd(t, theta)
		om = r.Pitch.Rate(t, thetaDot(y, ax, ay))
	} else {
		pit = theta // velocity-aligned payload ⇒ α=0
		om = 0
	}
	X, Y, Mz := aeroForcesWith(at, part, pit, y, rho, a)
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
