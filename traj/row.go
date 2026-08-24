package traj

import (
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
