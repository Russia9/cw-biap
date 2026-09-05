package main

import (
	"fmt"
	"os"
	"traj"
	"traj/aero"

	na "github.com/Russia9/numerical-analysis"
)

const h = 0.1

// args[1] = rocket.json path
// args[2] = aero averages.csv path
// args[3] = output.csv path
func main() {
	if len(os.Args) != 4 {
		panic("wrong number of arguments")
	}

	// Loading r.json
	r, err := traj.LoadRocketJSON(os.Args[1])
	if err != nil {
		panic(fmt.Errorf("rocket.json: %w", err))
	}

	// Loading aero table
	aero, err := aero.LoadCSV(os.Args[2])
	if err != nil {
		panic(fmt.Errorf("aero: %w", err))
	}

	if err := os.MkdirAll("out", 0o755); err != nil {
		panic(err)
	}

	// Init model
	model := traj.InitModel(r, aero)
	tChar := make([]float64, 0)
	for i, st := range r.Stages {
		if i > 0 {
			tChar = append(tChar, tChar[i-1]+st.BurnTime)
		} else {
			tChar = append(tChar, st.BurnTime)
		}
	}
	stop := func(x float64, y ...float64) (half bool, stop bool) {
		h := traj.Altitude(y...)
		if traj.Eq(h, 0, 1e-9) {
			return false, true
		} else if h < 0 {
			return true, false
		}
		return false, false
	}

	// Run calculation
	res, err := na.RungeKuttaMethod(model, 0, []float64{0, 0, 0, 0}, tChar, h, stop)
	if err != nil {
		panic(fmt.Errorf("rk4: %w", err))
	}

	// Visualize the trajectory
	/*{
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
	}*/

	// Write the resulting CSV
	out, err := os.Create(os.Args[3])
	if err != nil {
		panic(fmt.Errorf("out file: %w", err))
	}
	defer out.Close()

	out.WriteString("t,m,x,y,Vx,Vy,V,r,h,pitch,flightAngle,attack\n")
	for i := range res[0] {
		t := res[0][i].X                   // Current time
		stI := traj.StageIndex(r, true, t) // Stage index
		st := r.Stages[stI]                // Stage

		// State slice
		y := make([]float64, len(res))
		for j := range res {
			y[j] = res[j][i].Y
		}

		// pitch and attack
		pitch := r.Pitch.Pitch(t)
		attack := r.Pitch.Pitch(t) - traj.FlightAngle(y...)
		if !st.Controlled {
			pitch = traj.FlightAngle(y...)
			attack = 0
		}

		fmt.Fprintf(out, "%.3f,%e,%e,%e,%e,%e,%e,%e,%e,%e,%e,%e\n",
			t,
			traj.Mass(st, t, traj.StageT0(r, stI)),
			res[traj.IX][i].Y, res[traj.IY][i].Y,
			res[traj.IVx][i].Y, res[traj.IVy][i].Y,
			traj.VelMag(y...),
			traj.RadiusVec(y...),
			traj.Altitude(y...),
			pitch,
			traj.FlightAngle(y...),
			attack,
		)
	}
}
