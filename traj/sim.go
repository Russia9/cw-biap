package traj

import (
	"math"

	na "github.com/Russia9/numerical-analysis"
)

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
func stopAtTime(tEnd float64) func(t float64, y ...float64) (bool, bool) {
	return func(t float64, y ...float64) (bool, bool) {
		done := t >= tEnd-1e-9 || Altitude(y) < 0
		return done, done
	}
}

func stopGround(tMax float64) func(t float64, y ...float64) (bool, bool) {
	return func(t float64, y ...float64) (bool, bool) {
		H := Altitude(y)
		if t >= tMax {
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
