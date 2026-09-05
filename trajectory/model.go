package traj

import (
	"math"
	"traj/aero"
	"traj/atmosphere"

	na "github.com/Russia9/numerical-analysis"
)

// State layout
const (
	IVx = 0
	IVy = 1
	IX  = 2
	IY  = 3
)

// Radius-vector R
func RadiusVec(y ...float64) float64 {
	return math.Hypot(y[IX], Rz+y[IY])
}

// Altitude H = R − Rz [m].
func Altitude(y ...float64) float64 { return RadiusVec(y...) - Rz }

// VelMag returns |V| [m/s].
func VelMag(y ...float64) float64 { return math.Hypot(y[IVx], y[IVy]) }

// FlightAngle theta = atan2(Vy, Vx) [rad].
func FlightAngle(y ...float64) float64 { return math.Atan2(y[IVy], y[IVx]) }

// gravity components of the central field
func gravity(y ...float64) (gx, gy float64) {
	r := RadiusVec(y...)
	r3 := r * r * r
	gx = -MuZ * y[IX] / r3
	gy = -MuZ * (Rz + y[IY]) / r3
	return gx, gy
}

// StageIndex determines current StageIndex from time.
// final Stage is carried on regardless of BurnTime.
func StageIndex(r Rocket, fromRight bool, t float64) int {
	time := 0.
	for i, st := range r.Stages {
		if Eq(time+st.BurnTime, t, 1e-10) { // if we are at the stage separation event
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

func StageT0(r Rocket, stageIndex int) float64 {
	t0 := 0.
	for i, cur := range r.Stages {
		if i < stageIndex {
			t0 += cur.BurnTime
		}
	}
	return t0
}

func Thrust(st Stage, p float64) float64 {
	return ((st.ISpSurface-st.ISpVacuum)/P0*p + st.ISpVacuum) * st.MassFlow() * G0
}

func Mass(st Stage, t, t0 float64) float64 {
	if st.Powered {
		return st.M0 - st.MassFlow()*(t-t0)
	}
	return st.M0
}

func accel(r Rocket, aero map[string]*aero.Aero, fromRight bool, t float64, y ...float64) (ax, ay float64) {
	stI := StageIndex(r, fromRight, t)
	st := r.Stages[stI]

	// calculate stage start time
	t0 := StageT0(r, stI)

	// mass
	m := Mass(st, t, t0)

	// atmosphere
	_, rho, p, _, _, a := atmosphere.Atmosphere(Altitude(y...))

	// thrust
	if st.Powered {
		pitch := r.Pitch.Pitch(t)
		P := Thrust(st, p)
		ax += P * math.Cos(pitch) / m
		ay += P * math.Sin(pitch) / m
	}

	// aero
	if rho >= 0 && Altitude(y...) < Hatm {
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
		ax += (-X*math.Cos(theta) - Y*math.Sin(theta)) / m
		ay += (-X*math.Sin(theta) + Y*math.Cos(theta)) / m
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
		func(_ bool, _ float64, y ...float64) float64 { return y[IVx] }, // dx/dt
		func(_ bool, _ float64, y ...float64) float64 { return y[IVy] }, // dy/dt
	}
}
