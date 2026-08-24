package traj

import (
	_ "embed"
	"encoding/json"
	"fmt"
	"os"
)

// defaultConfigJSON is the built-in rocket spec used when no -config is given.
// It is the same file the optimizer edits, so the default run and an explicit
// `-config=rocket.json` produce identical output.
//
//go:embed rocket.json
var defaultConfigJSON []byte

// jsonArc is one pitch arc in the config. Exactly one of theta_deg (on a
// "theta"-steered stage) or alpha_deg (on an "alpha"-steered stage) is required;
// t_end is required only for non-final arcs of a stage (the final arc ends at
// stage burnout). Shape defaults to "exp"; K defaults per shape (see defaultKExp
// / defaultKCos). Theta0Deg is the explicit entry pitch a stage needs when it
// switches steering frame, where chaining from the previous arc is meaningless.
type jsonArc struct {
	TEnd      *float64 `json:"t_end,omitempty"`
	ThetaDeg  *float64 `json:"theta_deg,omitempty"`
	AlphaDeg  *float64 `json:"alpha_deg,omitempty"`
	Theta0Deg *float64 `json:"theta0_deg,omitempty"`
	Shape     string   `json:"shape,omitempty"`
	K         *float64 `json:"k,omitempty"`
}

type jsonStage struct {
	M0       float64   `json:"m0"`
	MFuel    float64   `json:"m_fuel"`
	BurnTime float64   `json:"burn_time"`
	IspSL    float64   `json:"isp_sl"`
	IspVac   float64   `json:"isp_vac"`
	AeroPart string    `json:"part"`
	Steering string    `json:"steering,omitempty"`
	Pitch    []jsonArc `json:"pitch"`
}

type jsonConfig struct {
	PayloadMass float64     `json:"payload_mass"`
	PayloadPart string      `json:"payload_part"`
	TVertical   float64     `json:"t_vertical"`
	Stages      []jsonStage `json:"stages"`
	Limits      Limits      `json:"limits"`
}

// DefaultConfig returns the built-in rocket and reporting limits.
func DefaultConfig() (Rocket, Limits) {
	r, lim, err := parseConfig(defaultConfigJSON)
	if err != nil {
		panic("embedded rocket.json invalid: " + err.Error())
	}
	return r, lim
}

// LoadConfig reads and parses a rocket JSON config file.
func LoadConfig(path string) (Rocket, Limits, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return Rocket{}, Limits{}, err
	}
	return parseConfig(data)
}

// parseConfig converts the JSON spec into the internal Rocket, flattening each
// stage's arcs into one continuous PitchProgram. The final arc of a stage ends at
// that stage's cumulative burnout time.
func parseConfig(data []byte) (Rocket, Limits, error) {
	var c jsonConfig
	if err := json.Unmarshal(data, &c); err != nil {
		return Rocket{}, Limits{}, fmt.Errorf("parse config: %w", err)
	}
	if len(c.Stages) == 0 {
		return Rocket{}, Limits{}, fmt.Errorf("config: no stages")
	}

	stages := make([]Stage, len(c.Stages))
	var segs []PitchSegment
	cum := 0.0
	for i, js := range c.Stages {
		stages[i] = Stage{
			M0: js.M0, MFuel: js.MFuel, BurnTime: js.BurnTime,
			IspSL: js.IspSL, IspVac: js.IspVac, AeroPart: js.AeroPart,
		}
		cum += js.BurnTime
		frame, err := stageFrame(js.Steering)
		if err != nil {
			return Rocket{}, Limits{}, fmt.Errorf("stage %d: %w", i+1, err)
		}
		for j, arc := range js.Pitch {
			shape, k, err := arcShape(arc)
			if err != nil {
				return Rocket{}, Limits{}, fmt.Errorf("stage %d arc %d: %w", i+1, j+1, err)
			}
			val, err := arcValue(arc, frame)
			if err != nil {
				return Rocket{}, Limits{}, fmt.Errorf("stage %d arc %d: %w", i+1, j+1, err)
			}
			tEnd := cum
			if arc.TEnd != nil {
				tEnd = *arc.TEnd
			} else if j != len(js.Pitch)-1 {
				return Rocket{}, Limits{}, fmt.Errorf("stage %d arc %d: non-final arc needs t_end", i+1, j+1)
			}
			seg := PitchSegment{TEnd: tEnd, Val: val, Shape: shape, Frame: frame, K: k}
			if arc.Theta0Deg != nil {
				entry := deg(*arc.Theta0Deg)
				seg.Entry = &entry
			}
			segs = append(segs, seg)
		}
	}

	r := Rocket{
		Stages:      stages,
		Payload:     c.PayloadMass,
		PayloadPart: c.PayloadPart,
		Pitch:       PitchProgram{TVert: c.TVertical, Segments: segs},
	}
	if err := r.Pitch.Validate(); err != nil {
		return Rocket{}, Limits{}, err
	}
	return r, c.Limits, nil
}

// stageFrame resolves a stage's steering frame. The empty string is "theta", so
// configs written before alpha steering existed parse unchanged.
func stageFrame(s string) (Frame, error) {
	switch s {
	case "", "theta":
		return FrameTheta, nil
	case "alpha":
		return FrameAlpha, nil
	default:
		return 0, fmt.Errorf("unknown steering %q (want \"theta\" or \"alpha\")", s)
	}
}

// arcValue picks the arc's terminal value [rad] from the field matching the
// stage's frame. The wrong field is an error rather than a silent default: a
// theta_deg on an alpha-steered stage would otherwise steer to a plausible but
// entirely different attitude.
func arcValue(a jsonArc, f Frame) (float64, error) {
	if f == FrameAlpha {
		if a.AlphaDeg == nil {
			return 0, fmt.Errorf("alpha-steered arc needs alpha_deg")
		}
		if a.ThetaDeg != nil {
			return 0, fmt.Errorf("alpha-steered arc must not set theta_deg")
		}
		return deg(*a.AlphaDeg), nil
	}
	if a.ThetaDeg == nil {
		return 0, fmt.Errorf("theta-steered arc needs theta_deg")
	}
	if a.AlphaDeg != nil {
		return 0, fmt.Errorf("theta-steered arc must not set alpha_deg")
	}
	return deg(*a.ThetaDeg), nil
}

// arcShape resolves an arc's shape and exponent, applying per-shape defaults.
func arcShape(a jsonArc) (Shape, float64, error) {
	switch a.Shape {
	case "", "exp":
		k := defaultKExp
		if a.K != nil {
			k = *a.K
		}
		return ShapeExp, k, nil
	case "cos":
		k := defaultKCos
		if a.K != nil {
			k = *a.K
		}
		return ShapeCos, k, nil
	default:
		return 0, 0, fmt.Errorf("unknown shape %q (want \"exp\" or \"cos\")", a.Shape)
	}
}
