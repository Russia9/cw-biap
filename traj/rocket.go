package traj

import (
	"encoding/json"
	"fmt"
	"io"
	"os"
	"slices"
)

type Rocket struct {
	Stages []Stage `json:"stages"`
	Pitch  Pitch   `json:"pitch"`
}

type Stage struct {
	M0         float64 `json:"m0"`
	MFuel      float64 `json:"m_fuel"`
	BurnTime   float64 `json:"burn_time"`
	ISpSurface float64 `json:"isp_sl"`  // P_уд.атм
	ISpVacuum  float64 `json:"isp_vac"` // P_уд.0

	Dm       float64 `json:"dm"` // middle diameter
	AeroPart string  `json:"part"`

	Powered    bool `json:"powered"`    // true = thrust and mass equations are enabled; false = ISpSurface and ISpVacuum are ignored.
	Controlled bool `json:"controlled"` // if true, the pitch program is applied; if false - pitch is defined so alpha=0.
	// If stage is powered, it must be controlled.
}

// MassFlow returns the constant beta=MFuel/BurnTime [kg/s]
func (s Stage) MassFlow() float64 { return s.MFuel / s.BurnTime }

func LoadRocketJSON(path string) (Rocket, error) {
	f, err := os.Open(path)
	if err != nil {
		return Rocket{}, fmt.Errorf("file: %w", err)
	}
	defer f.Close()

	raw, err := io.ReadAll(f)
	if err != nil {
		return Rocket{}, fmt.Errorf("io: %w", err)
	}

	var r Rocket
	err = json.Unmarshal(raw, &r)
	if err != nil {
		return Rocket{}, fmt.Errorf("json: %w", err)
	}

	for _, st := range r.Stages {
		if st.Powered && !st.Controlled {
			return Rocket{}, fmt.Errorf("stage is powered but not controlled")
		}
	}

	// Sort arcs by TEnd (asecnding)
	slices.SortFunc(r.Pitch.Segments, func(a, b PitchSegment) int {
		if a.TEnd > b.TEnd {
			return 1
		} else if a.TEnd == b.TEnd {
			return 0
		} else {
			return -1
		}
	})

	return r, nil
}
