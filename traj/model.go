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
	_, rho, _, _, _, a := atmosphere.Atmosphere(Altitude(y))
	return aeroForcesWith(at, part, pitch, y, rho, a)
}

// aeroForcesWith is AeroForces with the atmosphere (ρ, speed of sound) already
// evaluated, so a caller that needs the atmosphere for other terms too — or has
// it at hand anyway — pays for one lookup instead of several.
func aeroForcesWith(at *AeroTable, part string, pitch float64, y []float64, rho, a float64) (X, Y, Mz float64) {
	if rho <= 0 {
		return 0, 0, 0
	}
	V := VelMag(y)
	q := 0.5 * rho * V * V
	mach := 0.0
	if a > 0 {
		mach = V / a
	}
	alphaDeg := (pitch - FlightAngle(y)) * r2d
	cd, cl, cm := at.Coeffs(part, mach, alphaDeg)
	X = cd * q * Aref
	Y = cl * q * Aref
	Mz = cm * q * Aref * Lref
	return X, Y, Mz
}

// activeAccel returns the translational acceleration (ax, ay) on the powered leg
// at time t and state y: programmed pitch ϑ_пр(t), thrust, aerodynamics (whose
// magnitude follows ρ(H), so the atmosphere/vacuum split is automatic) and
// gravity. Shared by the dVx/dVy components so both read the same forces.
func (r Rocket) activeAccel(st Stage, at *AeroTable, t float64, y []float64) (ax, ay float64) {
	th := FlightAngle(y)
	pit := r.Pitch.Cmd(t, th)
	_, rho, p, _, _, a := atmosphere.Atmosphere(Altitude(y))
	X, Y, _ := aeroForcesWith(at, st.AeroPart, pit, y, rho, a)
	P := thrustWith(st, p)
	gx, gy := gravity(y)
	m := y[iM]
	ax = (P*math.Cos(pit)-X*math.Cos(th)-Y*math.Sin(th))/m + gx
	ay = (P*math.Sin(pit)-X*math.Sin(th)+Y*math.Cos(th))/m + gy
	return ax, ay
}

// thetaDot returns the flight-path rate θ̇ = (Vx·ay − Vy·ax)/V² [rad/s] implied
// by the translational acceleration (ax, ay). It closes the ϑ̇ bookkeeping for
// FrameAlpha steering, where ϑ̇ = θ̇ + α̇_пр. There is no circularity: α_пр is
// explicit in t, so ϑ → forces → θ̇ → ϑ̇ resolves in one forward pass.
func thetaDot(y []float64, ax, ay float64) float64 {
	v2 := y[iVx]*y[iVx] + y[iVy]*y[iVy]
	if v2 <= 0 {
		return 0
	}
	return (y[iVx]*ay - y[iVy]*ax) / v2
}

// accelCache memoizes one (t, state) → (ax, ay) evaluation. The RK library
// evaluates every ODE component as its own closure, handing the dVx and dVy
// components of one stage evaluation identical inputs — so without the cache
// the shared acceleration (atmosphere, aero table, thrust, gravity) would be
// computed twice per RK stage. A hit returns the float64s the identical code
// path already produced, so caching cannot change a single output bit. A NaN
// state never matches itself and simply recomputes.
type accelCache struct {
	ok     bool
	t      float64
	y      [5]float64
	ax, ay float64
}

func (c *accelCache) hit(t float64, y []float64) bool {
	return c.ok && c.t == t && c.y == [5]float64{y[0], y[1], y[2], y[3], y[4]}
}

func (c *accelCache) store(t float64, y []float64, ax, ay float64) {
	c.t, c.y = t, [5]float64{y[0], y[1], y[2], y[3], y[4]}
	c.ax, c.ay = ax, ay
	c.ok = true
}

// activeSystem builds the powered-flight ODE system for one stage (5 states).
// The returned system shares one accelCache and is not safe for concurrent use.
func (r Rocket) activeSystem(st Stage, at *AeroTable) na.FuncSystem {
	mdot := st.MassFlow()
	var c accelCache
	accel := func(t float64, y []float64) (float64, float64) {
		if c.hit(t, y) {
			return c.ax, c.ay
		}
		ax, ay := r.activeAccel(st, at, t, y)
		c.store(t, y, ax, ay)
		return ax, ay
	}
	return na.FuncSystem{
		func(_ bool, t float64, y ...float64) float64 { ax, _ := accel(t, y); return ax },
		func(_ bool, t float64, y ...float64) float64 { _, ay := accel(t, y); return ay },
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
// translation only). Shares one accelCache like activeSystem; the passive accel
// does not depend on time, but keying the cache on t as well stays correct.
func (r Rocket) passiveSystem(at *AeroTable) na.FuncSystem {
	var c accelCache
	accel := func(t float64, y []float64) (float64, float64) {
		if c.hit(t, y) {
			return c.ax, c.ay
		}
		ax, ay := r.passiveAccel(at, y)
		c.store(t, y, ax, ay)
		return ax, ay
	}
	return na.FuncSystem{
		func(_ bool, t float64, y ...float64) float64 { ax, _ := accel(t, y); return ax },
		func(_ bool, t float64, y ...float64) float64 { _, ay := accel(t, y); return ay },
		func(_ bool, _ float64, y ...float64) float64 { return y[iVx] }, // dx/dt
		func(_ bool, _ float64, y ...float64) float64 { return y[iVy] }, // dy/dt
		func(_ bool, _ float64, _ ...float64) float64 { return 0 },      // dm/dt
	}
}

// thrustWith returns engine thrust P = Isp(p)·β·g0 at ambient pressure p.
func thrustWith(st Stage, p float64) float64 {
	return ispPressure(st, p) * st.MassFlow() * G0
}
