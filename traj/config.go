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
// Thrust altitude correction (§4.2): P(p) = Isp(p)*MassFlow()*G0, with Isp(p)
// linear between sea level (IspSL = design impulse P_уд.р) and vacuum (IspVac =
// P_уд.п). These two anchors reproduce main.py's launch and vacuum thrusts
// exactly (e.g. stage 1: 694.8 kN and 750.6 kN).
type Stage struct {
	M0       float64 // sub-rocket launch mass [kg]
	MFuel    float64 // propellant mass ω_з [kg]
	BurnTime float64 // Δt_к [s]
	IspSL    float64 // specific impulse at sea level (P_уд.р) [s]
	IspVac   float64 // specific impulse in vacuum (P_уд.п) [s]
	MotorDia float64 // motor diameter [m]
	AeroPart string  // aerodynamic part key in averages.csv
}

// MassFlow returns the constant second-mass-flow β = MFuel/BurnTime [kg/s].
func (s Stage) MassFlow() float64 { return s.MFuel / s.BurnTime }

const (
	// Reference geometry shared by all aerodynamic coefficients in
	// averages.csv (openfoam gen_case.py: Aref = π·R_all², lRef = L_all from
	// the full-rocket STL bounding box; CofR at the nose).
	RrefAll = 0.79   // full-rocket max radius [m] -> Aref = π·0.79² = 1.96 m²
	Lref    = 18.293 // full-rocket length [m]
)

// Aref is the reference area for aerodynamic forces/moments [m²].
var Aref = math.Pi * RrefAll * RrefAll

// DragScale multiplies the CFD drag coefficient. Set to 0.9 to use 90 % of the
// tabulated Cd (sensitivity study against the openfoam coefficients).
const DragScale = 1

// Limits are the constructive-ballistic reporting thresholds (§4.4). The
// simulator measures the achieved maxima and flags whether each limit is met.
type Limits struct {
	Eps1        float64 `json:"eps1"`          // |α| limit for M ≤ 1.1 [deg]
	Eps2        float64 `json:"eps2"`          // |α| limit for M > 1.1 and H ≤ Hatm [deg]
	ThetaDotMax float64 `json:"theta_dot_max"` // |ϑ̇| limit [deg/s]
	Qmax        float64 `json:"qmax"`          // dynamic-pressure limit [Pa]
}

// Rocket bundles the full configuration handed to the simulator: the stage
// design figures, the payload, and the flattened per-stage pitch program.
type Rocket struct {
	Stages      []Stage
	Payload     float64
	PayloadPart string
	Pitch       PitchProgram
}

// BurnoutTimes returns the cumulative stage burnout (= staging) times [s].
func (r Rocket) BurnoutTimes() []float64 {
	tk := make([]float64, len(r.Stages))
	t := 0.0
	for i, st := range r.Stages {
		t += st.BurnTime
		tk[i] = t
	}
	return tk
}

func deg(d float64) float64 { return d * math.Pi / 180 }
