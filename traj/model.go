package traj

import (
	"math"
	"traj/aero"
	"traj/atmosphere"

	na "github.com/Russia9/numerical-analysis"
)

// State layout
const (
	iVx = 0
	iVy = 1
	iX  = 2
	iY  = 3
	iM  = 4
)

// altitude H = R − Rz [m].
func altitude(y ...float64) float64 { return math.Hypot(y[iX], Rz+y[iY]) - Rz }

// VelMag returns |V| [m/s].
func VelMag(y ...float64) float64 { return math.Hypot(y[iVx], y[iVy]) }

// FlightAngle theta = atan2(Vy, Vx) [rad].
func FlightAngle(y ...float64) float64 { return math.Atan2(y[iVy], y[iVx]) }

// gravity components of the central field
func gravity(y ...float64) (gx, gy float64) {
	r := math.Hypot(y[iX], Rz+y[iY])
	r3 := r * r * r
	gx = -MuZ * y[iX] / r3
	gy = -MuZ * (Rz + y[iY]) / r3
	return gx, gy
}

// stage determines current stage from time.
// final stage is carried on regardless of BurnTime.
func stage(r Rocket, fromRight bool, t float64) int {
	time := 0.
	for i, st := range r.Stages {
		if eq(time+st.BurnTime, t) { // if we are at the stage separation event
			if fromRight {
				return i + 1
			}
			return i
		}

		if time+st.BurnTime >= t {
			return i
		}
		time += st.BurnTime
	}

	return len(r.Stages) - 1
}

func thrust(st Stage, p float64) float64 {
	return ((st.ISpSurface-st.ISpVacuum)/P0*p + st.ISpVacuum) * st.MassFlow() * G0
}

func accel(r Rocket, aero map[string]*aero.Aero, fromRight bool, t float64, y ...float64) (ax, ay float64) {
	st := r.Stages[stage(r, fromRight, t)]

	// atmosphere
	_, rho, p, _, _, a := atmosphere.Atmosphere(altitude(y...))

	// thrust
	if st.Powered {
		pitch := r.Pitch.Pitch(t)
		P := thrust(st, p)
		ax += P * math.Cos(pitch) / y[iM]
		ay += P * math.Sin(pitch) / y[iM]
	}

	// aero
	if rho >= 0 && altitude(y...) < Hatm {
		theta := FlightAngle(y...)
		V := VelMag(y...)
		q := 0.5 * rho * V * V
		mach := 0.0
		if a > 0 {
			mach = V / a
		}
		alphaDeg := 0.
		if st.Controlled {
			alphaDeg = (r.Pitch.Pitch(t) - theta) * r2d
		}
		X := aero[st.AeroPart].Cd(mach, alphaDeg) * q * Aref
		Y := aero[st.AeroPart].Cl(mach, alphaDeg) * q * Aref
		ax += (-X*math.Cos(theta) - Y*math.Sin(theta)) / y[iM]
		ay += (-X*math.Sin(theta) + Y*math.Cos(theta)) / y[iM]
	}

	// gravity
	gx, gy := gravity(y...)
	ax += gx
	ay += gy

	return ax, ay
}

func InitModel(r Rocket, aero map[string]*aero.Aero) na.FuncSystem {
	return na.FuncSystem{
		func(fromRight bool, t float64, y ...float64) float64 {
			ax, _ := accel(r, aero, fromRight, t, y...)
			return ax
		}, // dVx/dt
		func(fromRight bool, t float64, y ...float64) float64 {
			_, ay := accel(r, aero, fromRight, t, y...)
			return ay
		}, // dVy/dt
		func(_ bool, _ float64, y ...float64) float64 { return y[iVx] }, // dx/dt
		func(_ bool, _ float64, y ...float64) float64 { return y[iVy] }, // dy/dt
		func(fromRight bool, t float64, _ ...float64) float64 {
			return -1 * r.Stages[stage(r, fromRight, t)].MassFlow()
		}, // dm/dt
	}
}
