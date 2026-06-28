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

// pudPressure interpolates specific impulse with ambient pressure (§4.2):
// linear between vacuum (p=0 → PudVac) and sea level (p=P0 → PudSL).
func pudPressure(st Stage, p float64) float64 {
	return (st.PudSL-st.PudVac)/P0*p + st.PudVac
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
	X = cd * q * Aref
	Y = cl * q * Aref
	Mz = cm * q * Aref * Lref
	return X, Y, Mz
}

// activeSystem builds the powered-flight ODE system for one stage (5 states).
// Pitch is the programmed ϑ_пр(t); aerodynamic magnitude follows ρ(H), so the
// atmosphere/vacuum split is automatic.
func (r Rocket) activeSystem(st Stage, at *AeroTable) na.FuncSystem {
	mdot := st.MassFlow()
	return na.FuncSystem{
		func(_ bool, x float64, y ...float64) float64 { // dVx/dt
			pit := r.Pitch.Theta(x)
			th := FlightAngle(y)
			X, Y, _ := AeroForces(at, st.Part, pit, y)
			P := thrustOf(st, y)
			gx, _ := gravity(y)
			return (P*math.Cos(pit)-X*math.Cos(th)-Y*math.Sin(th))/y[iM] + gx
		},
		func(_ bool, x float64, y ...float64) float64 { // dVy/dt
			pit := r.Pitch.Theta(x)
			th := FlightAngle(y)
			X, Y, _ := AeroForces(at, st.Part, pit, y)
			P := thrustOf(st, y)
			_, gy := gravity(y)
			return (P*math.Sin(pit)-X*math.Sin(th)+Y*math.Cos(th))/y[iM] + gy
		},
		func(_ bool, _ float64, y ...float64) float64 { return y[iVx] }, // dx/dt
		func(_ bool, _ float64, y ...float64) float64 { return y[iVy] }, // dy/dt
		func(_ bool, _ float64, _ ...float64) float64 { return -mdot },  // dm/dt
	}
}

// passiveSystem builds the unpowered-flight ODE system for the payload (5 states,
// translation only). The payload is assumed aerodynamically velocity-aligned
// (α≡0): pitch = flight-path angle θ, so AeroForces yields Y=0 and Mz=0 and only
// drag acts, opposing the velocity vector. This mirrors the C++ reference and
// avoids the non-physical free-attitude tumble on re-entry.
func (r Rocket) passiveSystem(at *AeroTable) na.FuncSystem {
	part := PayloadPart
	return na.FuncSystem{
		func(_ bool, _ float64, y ...float64) float64 { // dVx/dt
			th := FlightAngle(y)
			X, _, _ := AeroForces(at, part, th, y) // pitch=θ ⇒ α=0 ⇒ Y=0
			gx, _ := gravity(y)
			return -X*math.Cos(th)/y[iM] + gx
		},
		func(_ bool, _ float64, y ...float64) float64 { // dVy/dt
			th := FlightAngle(y)
			X, _, _ := AeroForces(at, part, th, y)
			_, gy := gravity(y)
			return -X*math.Sin(th)/y[iM] + gy
		},
		func(_ bool, _ float64, y ...float64) float64 { return y[iVx] }, // dx/dt
		func(_ bool, _ float64, y ...float64) float64 { return y[iVy] }, // dy/dt
		func(_ bool, _ float64, _ ...float64) float64 { return 0 },      // dm/dt
	}
}

// thrustOf returns engine thrust P = Pud(p)·β·g0 at the current altitude.
func thrustOf(st Stage, y []float64) float64 {
	_, _, p, _, _, _ := atmosphere.Atmosphere(Altitude(y))
	return pudPressure(st, p) * st.MassFlow() * G0
}
