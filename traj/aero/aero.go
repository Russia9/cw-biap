package aero

import (
	"fmt"
	"os"

	na "github.com/Russia9/numerical-analysis"
	"github.com/gocarina/gocsv"
)

type csvRow struct {
	Part    string  `csv:"part"`
	Ma      float64 `csv:"Ma"`
	Alpha   float64 `csv:"alpha"`
	Cd      float64 `csv:"Cd_mean"`
	Cl      float64 `csv:"Cl_mean"`
	CmPitch float64 `csv:"CmPitch_mean"`
}

type AeroCoefficient func(Ma, alphaDeg float64) float64

type Aero struct {
	Cd, Cl, CmPitch AeroCoefficient
}

func LoadCSV(path string) (map[string]*Aero, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, fmt.Errorf("file: %w", err)
	}
	defer f.Close()

	var rows []csvRow
	if err := gocsv.UnmarshalFile(f, &rows); err != nil {
		return nil, fmt.Errorf("parse: %w", err)
	}

	rawCd := make(map[string][]na.Point3D, 0)
	rawCl := make(map[string][]na.Point3D, 0)
	rawCmPitch := make(map[string][]na.Point3D, 0)

	for _, row := range rows {
		if rawCd[row.Part] == nil {
			rawCd[row.Part] = make([]na.Point3D, 0)
			rawCl[row.Part] = make([]na.Point3D, 0)
			rawCmPitch[row.Part] = make([]na.Point3D, 0)
		}

		rawCd[row.Part] = append(rawCd[row.Part], na.Point3D{X: row.Ma, Y: row.Alpha, Z: row.Cd})
		rawCl[row.Part] = append(rawCl[row.Part], na.Point3D{X: row.Ma, Y: row.Alpha, Z: row.Cl})
		rawCmPitch[row.Part] = append(rawCmPitch[row.Part], na.Point3D{X: row.Ma, Y: row.Alpha, Z: row.CmPitch})
	}

	res := make(map[string]*Aero, len(rawCd))
	for part := range rawCd {
		res[part] = &Aero{
			Cd:      AeroCoefficient(na.BilinearInterpolation2D(rawCd[part])),
			Cl:      AeroCoefficient(na.BilinearInterpolation2D(rawCl[part])),
			CmPitch: AeroCoefficient(na.BilinearInterpolation2D(rawCmPitch[part])),
		}
	}

	return res, nil
}
