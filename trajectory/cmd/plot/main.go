package main

import (
	"fmt"
	"math"
	"traj"

	"gonum.org/v1/plot"
	"gonum.org/v1/plot/plotter"
	"gonum.org/v1/plot/vg"
)

func main() {
	// Loading rocket.json
	rocket, err := traj.LoadRocketJSON("rocket.json")
	if err != nil {
		panic(fmt.Errorf("rocket.json: %w", err))
	}

	p := plot.New()

	p.Title.Text = "Pitch program"
	p.X.Label.Text = "t, с"
	p.Y.Label.Text = "theta, deg"

	pitch := plotter.NewFunction(func(t float64) float64 { return rocket.Pitch.Pitch(t) * 180 / math.Pi })
	pitch.Samples = 1000
	p.Add(pitch)

	// Set the axis ranges.  Unlike other data sets,
	// functions don't set the axis ranges automatically
	// since functions don't necessarily have a
	// finite range of x and y values.
	p.X.Min = 0
	p.X.Max = 150
	p.Y.Min = 0
	p.Y.Max = 180

	// Save the plot to a PNG file.
	if err := p.Save(15*vg.Inch, 8*vg.Inch, "out/pitch.png"); err != nil {
		panic(err)
	}
}
