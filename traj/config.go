package traj

import "math"

// Physical constants (central-gravity, non-rotating spherical Earth — §4.1/§4.2).
const (
	Rz   = 6371000.0 // Earth radius [m]
	MuZ  = 3.986e14  // Earth gravitational parameter [m^3/s^2]
	G0   = 9.80665   // standard gravity [m/s^2]
	P0   = 101325.0  // sea-level pressure [Pa]
	Hatm = 94000.0   // conditional atmosphere boundary [m]
)

// Stage holds the per-stage design figures taken from main.py's output.
//
// Mass bookkeeping: M0 is the launch mass of the sub-rocket (this stage plus
// everything above it). During the burn, mass decreases by MassFlow()*BurnTime
// = MFuel. At separation the spent dry structure is dropped and the mass becomes
// the next stage's M0 (or the payload mass after the last stage).
//
// Thrust altitude correction (§4.2): P(p) = Pud(p)*MassFlow()*G0, with
// Pud(p) linear between sea level (PudSL = design impulse P_уд.р) and vacuum
// (PudVac = P_уд.п). These two anchors reproduce main.py's launch and vacuum
// thrusts exactly (e.g. stage 1: 694.8 kN and 750.6 kN).
type Stage struct {
	M0       float64 // sub-rocket launch mass [kg]
	MFuel    float64 // propellant mass ω_з [kg]
	BurnTime float64 // Δt_к [s]
	PudSL    float64 // specific impulse at sea level (P_уд.р) [s]
	PudVac   float64 // specific impulse in vacuum (P_уд.п) [s]
	Dm       float64 // motor diameter [m]
	Part     string  // aerodynamic part key in averages.csv
}

// MassFlow returns the constant second-mass-flow β = MFuel/BurnTime [kg/s].
func (s Stage) MassFlow() float64 { return s.MFuel / s.BurnTime }

// Stages is the three-stage configuration (source: main.py).
var Stages = []Stage{
	{M0: 30568, MFuel: 20101, BurnTime: 66.4, PudSL: 233.95, PudVac: 252.75, Dm: 1.58, Part: "all"},
	{M0: 8702, MFuel: 5722, BurnTime: 43.0, PudSL: 244.02, PudVac: 265.75, Dm: 1.17, Part: "stage2up"},
	{M0: 2476, MFuel: 1628, BurnTime: 30.8, PudSL: 257.18, PudVac: 275.22, Dm: 0.84, Part: "stage3up"},
}

const (
	// Payload = warhead + control unit (M_BCH + M_AU from main.py).
	PayloadMass = 620.0
	PayloadPart = "head"

	// Reference geometry shared by all aerodynamic coefficients in
	// averages.csv (openfoam gen_case.py: Aref = π·R_all², lRef = L_all from
	// the full-rocket STL bounding box; CofR at the nose).
	RrefAll = 0.79   // full-rocket max radius [m] -> Aref = π·0.79² = 1.96 m²
	Lref    = 18.293 // full-rocket length [m]
)

// Aref is the reference area for aerodynamic forces/moments [m²].
var Aref = math.Pi * RrefAll * RrefAll

// PitchParams defines the 4-phase pitch program (§4.3). The free parameters
// tuned to satisfy the constructive-ballistic constraints are Tv and the three
// terminal pitch angles; the phase-end times Tk1/Tk2/Tk3 are the cumulative
// stage burn times.
type PitchParams struct {
	Tv               float64 // vertical-hold duration t_в [s]
	Tk1, Tk2, Tk3    float64 // stage burnout times [s]
	ThK1, ThK2, ThK3 float64 // terminal pitch angles ϑ_k1..k3 [rad]
}

// Rocket bundles the full configuration handed to the simulator.
type Rocket struct {
	Stages  []Stage
	Payload float64
	Pitch   PitchParams
}

func deg(d float64) float64 { return d * math.Pi / 180 }

// DefaultRocket wires the design figures with an initial (tunable) pitch
// program. The terminal angles and Tv are first guesses — adjust them using the
// constraint diagnostics printed by the simulator.
func DefaultRocket() Rocket {
	tk1 := Stages[0].BurnTime
	tk2 := tk1 + Stages[1].BurnTime
	tk3 := tk2 + Stages[2].BurnTime
	return Rocket{
		Stages:  Stages,
		Payload: PayloadMass,
		Pitch: PitchParams{
			Tv:   10,
			Tk1:  tk1,
			Tk2:  tk2,
			Tk3:  tk3,
			ThK1: deg(72),
			ThK2: deg(46),
			ThK3: deg(28),
		},
	}
}

// Constraint limits (§4.4). These are reporting thresholds — the simulator
// measures the achieved maxima and flags whether each limit is met. Set them to
// the values used in the report.
const (
	Eps1        = 2.0   // |α| limit for M ≤ 1.1 [deg]
	Eps2        = 8.0   // |α| limit for M > 1.1 and H ≤ Hatm [deg]
	ThetaDotMax = 3.0   // |ϑ̇| limit [deg/s]
	Qmax        = 120e3 // dynamic-pressure limit [Pa]
)
