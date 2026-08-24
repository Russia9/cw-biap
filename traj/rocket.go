package traj

// Stage holds the per-stage design figures taken from main.py's output.
//
// Mass bookkeeping: M0 is the launch mass of the sub-rocket (this stage plus
// everything above it). During the burn, mass decreases by MassFlow()*BurnTime
// = MFuel. At separation the spent dry structure is dropped and the mass becomes
// the next stage's M0 (or the payload mass after the last stage).
//
// Thrust altitude correction (§4.2): P(p) = Isp(p)*MassFlow()*G0, with Isp(p)
// linear between sea level (IspSL = ground impulse P_уд.0) and vacuum (IspVac =
// P_уд.п). These two anchors reproduce main.py's launch and vacuum thrusts
// exactly (stage 1 of the shipped rocket.json: 668.4 kN and 748.5 kN, from
// Isp·(m_fuel/burn_time)·g0).
type Stage struct {
	M0       float64 // sub-rocket launch mass [kg]
	MFuel    float64 // propellant mass ω_з [kg]
	BurnTime float64 // Δt_к [s]
	IspSL    float64 // specific impulse at sea level (P_уд.0) [s]
	IspVac   float64 // specific impulse in vacuum (P_уд.п) [s]
	MotorDia float64 // motor diameter [m] — informational; see RrefAll in constants.go
	AeroPart string  // aerodynamic part key (see fallbacks in aero.go)
}

// MassFlow returns the constant second-mass-flow β = MFuel/BurnTime [kg/s].
func (s Stage) MassFlow() float64 { return s.MFuel / s.BurnTime }

// Limits are the constructive-ballistic reporting thresholds (§4.4). The
// simulator measures the achieved maxima and flags whether each limit is met.
type Limits struct {
	Eps1 float64 `json:"eps1"` // |α| limit for M ≤ 1.1 [deg]
	Eps2 float64 `json:"eps2"` // |α| limit for M > 1.1 and H ≤ Hatm [deg]
	// PitchRateMax limits the body pitch rate |ϑ̇| [deg/s] — not the
	// flight-path rate θ̇ that the thetaDot helper computes. The JSON key
	// keeps its historical name; renaming it would break every config.
	PitchRateMax float64 `json:"theta_dot_max"`
	Qmax         float64 `json:"qmax"`  // dynamic-pressure limit [Pa]
	HMax         float64 `json:"h_max"` // max trajectory ordinate (apogee) [m]; 0 disables
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
