package traj

import (
	"math"

	na "github.com/Russia9/numerical-analysis"
	"traj/atmosphere"
)

// State vector layout (na uses a flat []float64 / variadic y). Both the active
// and passive legs use the same 5 states:
//
//	y[0] Vx   y[1] Vy   y[2] x   y[3] y   y[4] m
const (
	iVx = 0
	iVy = 1
	iX  = 2
	iY  = 3
	iM  = 4
)

// radius from Earth's centre: R = √(x² + (Rz+y)²)  (§4.2).
func radius(y []float64) float64 {
	return math.Hypot(y[iX], Rz+y[iY])
}

// Altitude H = R − Rz [m].
func Altitude(y []float64) float64 { return radius(y) - Rz }

// VelMag returns |V| [m/s].
func VelMag(y []float64) float64 { return math.Hypot(y[iVx], y[iVy]) }

// FlightAngle θ = atan2(Vy, Vx) [rad].
func FlightAngle(y []float64) float64 { return math.Atan2(y[iVy], y[iVx]) }

// gravity components of the central field (§4.2).
func gravity(y []float64) (gx, gy float64) {
	r := radius(y)
	r3 := r * r * r
	gx = -MuZ * y[iX] / r3
	gy = -MuZ * (Rz + y[iY]) / r3
	return gx, gy
}

// ispPressure interpolates specific impulse with ambient pressure (§4.2):
// linear between vacuum (p=0 → IspVac) and sea level (p=P0 → IspSL).
func ispPressure(st Stage, p float64) float64 {
	return (st.IspSL-st.IspVac)/P0*p + st.IspVac
}

// AeroForces returns drag X, lift Y [N] and pitch moment Mz [N·m] for the given
// part and body pitch ϑ. Y and Mz carry the sign of the angle of attack; X ≥ 0.
// In vacuum (ρ=0) all three are zero.
func AeroForces(at *AeroTable, part string, pitch float64, y []float64) (X, Y, Mz float64) {
	H := Altitude(y)
	_, rho, _, _, _, a := atmosphere.Atmosphere(H)
	if rho <= 0 {
		return 0, 0, 0
	}
	V := VelMag(y)
	q := 0.5 * rho * V * V
	mach := 0.0
	if a > 0 {
		mach = V / a
	}
	alphaDeg := (pitch - FlightAngle(y)) * 180 / math.Pi
	cd, cl, cm := at.Coeffs(part, mach, alphaDeg)
	X = DragScale * cd * q * Aref
	Y = cl * q * Aref
	Mz = cm * q * Aref * Lref
	return X, Y, Mz
}

// activeAccel returns the translational acceleration (ax, ay) on the powered leg
// at time x and state y: programmed pitch ϑ_пр(x), thrust, aerodynamics (whose
// magnitude follows ρ(H), so the atmosphere/vacuum split is automatic) and
// gravity. Shared by the dVx/dVy components so both read the same forces.
func (r Rocket) activeAccel(st Stage, at *AeroTable, x float64, y []float64) (ax, ay float64) {
	pit := r.Pitch.Theta(x)
	th := FlightAngle(y)
	X, Y, _ := AeroForces(at, st.AeroPart, pit, y)
	P := thrustOf(st, y)
	gx, gy := gravity(y)
	m := y[iM]
	ax = (P*math.Cos(pit)-X*math.Cos(th)-Y*math.Sin(th))/m + gx
	ay = (P*math.Sin(pit)-X*math.Sin(th)+Y*math.Cos(th))/m + gy
	return ax, ay
}

// activeSystem builds the powered-flight ODE system for one stage (5 states).
func (r Rocket) activeSystem(st Stage, at *AeroTable) na.FuncSystem {
	mdot := st.MassFlow()
	return na.FuncSystem{
		func(_ bool, x float64, y ...float64) float64 { ax, _ := r.activeAccel(st, at, x, y); return ax },
		func(_ bool, x float64, y ...float64) float64 { _, ay := r.activeAccel(st, at, x, y); return ay },
		func(_ bool, _ float64, y ...float64) float64 { return y[iVx] }, // dx/dt
		func(_ bool, _ float64, y ...float64) float64 { return y[iVy] }, // dy/dt
		func(_ bool, _ float64, _ ...float64) float64 { return -mdot },  // dm/dt
	}
}

// passiveAccel returns the unpowered translational acceleration (ax, ay) for the
// payload. The payload is assumed aerodynamically velocity-aligned (α≡0): pitch =
// flight-path angle θ, so AeroForces yields Y=0 and Mz=0 and only drag acts,
// opposing the velocity vector. This mirrors the C++ reference and avoids the
// non-physical free-attitude tumble on re-entry.
func (r Rocket) passiveAccel(at *AeroTable, y []float64) (ax, ay float64) {
	th := FlightAngle(y)
	X, _, _ := AeroForces(at, r.PayloadPart, th, y) // pitch=θ ⇒ α=0 ⇒ Y=0
	gx, gy := gravity(y)
	m := y[iM]
	ax = -X*math.Cos(th)/m + gx
	ay = -X*math.Sin(th)/m + gy
	return ax, ay
}

// passiveSystem builds the unpowered-flight ODE system for the payload (5 states,
// translation only).
func (r Rocket) passiveSystem(at *AeroTable) na.FuncSystem {
	return na.FuncSystem{
		func(_ bool, _ float64, y ...float64) float64 { ax, _ := r.passiveAccel(at, y); return ax },
		func(_ bool, _ float64, y ...float64) float64 { _, ay := r.passiveAccel(at, y); return ay },
		func(_ bool, _ float64, y ...float64) float64 { return y[iVx] }, // dx/dt
		func(_ bool, _ float64, y ...float64) float64 { return y[iVy] }, // dy/dt
		func(_ bool, _ float64, _ ...float64) float64 { return 0 },      // dm/dt
	}
}

// thrustOf returns engine thrust P = Isp(p)·β·g0 at the current altitude.
func thrustOf(st Stage, y []float64) float64 {
	_, _, p, _, _, _ := atmosphere.Atmosphere(Altitude(y))
	return ispPressure(st, p) * st.MassFlow() * G0
}
