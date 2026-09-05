package main

import (
	"fmt"
	"os"
	"traj"
	"traj/aero"

	na "github.com/Russia9/numerical-analysis"
	"gonum.org/v1/plot"
	"gonum.org/v1/plot/plotter"
	"gonum.org/v1/plot/plotutil"
	"gonum.org/v1/plot/vg"
)

const h = 0.1

// args[1] = rocket.json path
// args[2] = aero averages.csv path
func main() {
	if len(os.Args) != 3 {
		panic("wrong number of arguments")
	}

	// Loading rocket.json
	rocket, err := traj.LoadRocketJSON(os.Args[1])
	if err != nil {
		panic(fmt.Errorf("rocket.json: %w", err))
	}

	// Loading aero table
	aero, err := aero.LoadCSV(os.Args[2])
	if err != nil {
		panic(fmt.Errorf("aero: %w", err))
	}

	// Init model
	model := traj.InitModel(rocket, aero)
	tChar := make([]float64, 0)
	for i, st := range rocket.Stages {
		if i > 0 {
			tChar = append(tChar, tChar[i-1]+st.BurnTime)
		} else {
			tChar = append(tChar, st.BurnTime)
		}
	}
	stop := func(x float64, y ...float64) (half bool, stop bool) {
		h := traj.Altitude(y...)
		if traj.Eq(h, 0) {
			return false, true
		} else if h < 0 {
			return true, false
		}
		return false, false
	}

	// Run calculation
	res, err := na.RungeKuttaMethod(model, 0, []float64{0, 0, 0, 0, rocket.Stages[0].M0}, tChar, h, stop)
	if err != nil {
		panic(fmt.Errorf("rk4: %w", err))
	}

	// Visualize the trajectory
	{
		p := plot.New()

		p.Title.Text = "Trajectory"
		p.X.Label.Text = "x, m"
		p.Y.Label.Text = "y, m"

		pts := make(plotter.XYs, len(res[0]))
		for i := range res[0] {
			pts[i] = plotter.XY{X: res[2][i].Y, Y: res[3][i].Y}
		}
		err = plotutil.AddLinePoints(p, "trajectory", pts)
		if err != nil {
			panic(fmt.Errorf("plot: %w", err))
		}

		// Save the plot to a PNG file.
		if err := p.Save(15*vg.Inch, 8*vg.Inch, "out/trajectory.png"); err != nil {
			panic(err)
		}
	}
}
