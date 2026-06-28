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

// jsonArc is one pitch arc in the config. ThetaDeg is required; t_end is required
// only for non-final arcs of a stage (the final arc ends at stage burnout). Shape
// defaults to "exp"; K defaults per shape (see defaultKExp / defaultKCos).
type jsonArc struct {
	TEnd     *float64 `json:"t_end,omitempty"`
	ThetaDeg float64  `json:"theta_deg"`
	Shape    string   `json:"shape,omitempty"`
	K        *float64 `json:"k,omitempty"`
}

type jsonStage struct {
	M0       float64   `json:"m0"`
	MFuel    float64   `json:"m_fuel"`
	BurnTime float64   `json:"burn_time"`
	IspSL    float64   `json:"isp_sl"`
	IspVac   float64   `json:"isp_vac"`
	MotorDia float64   `json:"dm"`
	AeroPart string    `json:"part"`
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
			IspSL: js.IspSL, IspVac: js.IspVac, MotorDia: js.MotorDia, AeroPart: js.AeroPart,
		}
		cum += js.BurnTime
		for j, arc := range js.Pitch {
			shape, k, err := arcShape(arc)
			if err != nil {
				return Rocket{}, Limits{}, fmt.Errorf("stage %d arc %d: %w", i+1, j+1, err)
			}
			tEnd := cum
			if arc.TEnd != nil {
				tEnd = *arc.TEnd
			} else if j != len(js.Pitch)-1 {
				return Rocket{}, Limits{}, fmt.Errorf("stage %d arc %d: non-final arc needs t_end", i+1, j+1)
			}
			segs = append(segs, PitchSegment{TEnd: tEnd, Theta: deg(arc.ThetaDeg), Shape: shape, K: k})
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
