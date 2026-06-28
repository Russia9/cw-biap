package obla

import (
	"encoding/csv"
	"fmt"
	"image/color"
	"math"
	"os"

	na "github.com/Russia9/numerical-analysis"
	"gonum.org/v1/plot"
	"gonum.org/v1/plot/plotter"
	"gonum.org/v1/plot/vg"
	"gonum.org/v1/plot/vg/draw"
)

func GraphNewton(newton func(t float64) float64, t_around float64, deltaT float64, N int, filename string) {
	p := plot.New()

	p.Title.Text = "Newton's Method"
	p.X.Label.Text = "t"
	p.Y.Label.Text = "f(t)"
	p.X.Max = t_around + deltaT
	p.Y.Max = math.Max(math.Abs(newton(t_around+deltaT)), math.Abs(newton(t_around-deltaT)))

	p.Add(plotter.NewGrid())

	points := make(plotter.XYs, N*2)
	for i := range N * 2 {
		t := t_around - deltaT + float64(i)*deltaT/float64(N)
		points[i].X = t
		points[i].Y = newton(t)

		if i == N {
			sc, err := plotter.NewScatter(plotter.XYs{
				points[i],
			})
			if err != nil {
				panic(err)
			}
			sc.Color = color.RGBA{R: 255, G: 0, B: 0, A: 255}
			p.Add(sc)
		}
	}
	l, err := plotter.NewLine(points)
	if err != nil {
		panic(err)
	}
	p.Add(l)

	p.Save(22*vg.Centimeter, 12*vg.Centimeter, filename)
}

func GraphTrajectory(result [][]na.Point2D, tChar []float64, name, filename string) {
	p := plot.New()

	p.Title.Text = name
	p.X.Label.Text = "X, m"
	p.Y.Label.Text = "Y, m"
	p.X.Max = 10

	p.Add(plotter.NewGrid())

	points := make(plotter.XYs, len(result[2]))
	charPoints := make(plotter.XYs, len(tChar))
	for i := range result[2] {
		points[i].X = result[2][i].Y
		points[i].Y = result[3][i].Y

		for _, char := range tChar {
			if result[2][i].X == char {
				charPoints = append(charPoints, plotter.XY{X: result[2][i].Y, Y: result[3][i].Y})
			}
		}
	}
	l, err := plotter.NewLine(points)
	if err != nil {
		panic(err)
	}
	p.Add(l)

	c, err := plotter.NewScatter(charPoints)
	c.Color = color.RGBA{R: 255, G: 0, B: 0, A: 255}
	c.Shape = draw.CircleGlyph{}
	if err != nil {
		panic(err)
	}
	p.Add(c)

	p.Save(22*vg.Centimeter, 12*vg.Centimeter, filename)
}

func TrajParamsOutput(filename string, theta_1dot, theta_2, t_1, t_2, dif1, dif2 float64) {
	file, err := os.Create(filename)
	if err != nil {
		panic(err)
	}
	defer file.Close()

	fmt.Fprintf(file, "theta_1dot = %.6f\n", theta_1dot)
	fmt.Fprintf(file, "theta_2 = %.6f\n", theta_2)
	fmt.Fprintf(file, "t_1 = %.6f\n", t_1)
	fmt.Fprintf(file, "t_2 = %.6f\n", t_2)
	fmt.Fprintf(file, "dm_pg/dm = %.6f\n", dif1)
	fmt.Fprintf(file, "dm_pg/dW = %.6f\n", dif2)
}

func TypstTrajOutput(result [][]na.Point2D, filename string, t_r, t_1, t_2, theta_1dot, theta_2 float64) {
	file, err := os.Create(filename)
	if err != nil {
		panic(err)
	}
	defer file.Close()

	file.WriteString("t, с\tm, кг\tV_x, м/с\tV_y, м/с\tx, м\ty, м\th, м\tV, м/с\ttheta, град.\tTheta, град.\tTheta_с, град.\talpha, град.\tphi, град.\t")

	for i, _ := range result[0] {
		row := make([]string, 13)

		// t
		t := result[0][i].X
		if int(t*10)%500 != 0 && math.Abs(t-t_r) > 1e-6 && math.Abs(t-t_1) > 1e-6 && math.Abs(t-t_2) > 1e-6 && i != len(result[0])-1 {
			continue
		}
		row[0] = fmt.Sprintf("%.2f", t)
		// for j, col := range result {
		// 	row[j+1] = fmt.Sprintf("%.2f", col[i].Y)
		// }

		// m, Vx, Vy, X, Y
		Vx := result[0][i].Y
		Vy := result[1][i].Y
		X := result[2][i].Y
		Y := result[3][i].Y + R_M
		m := result[4][i].Y

		row[1] = fmt.Sprintf("%.2f", m)
		row[2] = fmt.Sprintf("%.3f", Vx)
		row[3] = fmt.Sprintf("%.3f", Vy)
		row[4] = fmt.Sprintf("%.2f", X/1000)
		row[5] = fmt.Sprintf("%.2f", Y/1000)

		// h, V, theta, Theta, Theta_s, alpha, phi
		R := math.Sqrt(X*X + Y*Y)
		h := R - R_M
		V := math.Sqrt(Vx*Vx + Vy*Vy)
		theta := theta(false, t, t_r, t_1, theta_1dot, theta_2)
		Theta := math.Asin((Vx*X + Vy*Y) / (V * R)) // to current horizon
		Theta_s := math.Atan(Vy / Vx)               // to starting horizon
		alpha := theta - Theta_s
		phi := math.Asin(X / R)

		row[6] = fmt.Sprintf("%.2f", h/1000)
		row[7] = fmt.Sprintf("%.3f", V)
		row[8] = fmt.Sprintf("%.3f", theta*180/math.Pi)
		row[9] = fmt.Sprintf("%.3f", Theta*180/math.Pi)
		row[10] = fmt.Sprintf("%.3f", Theta_s*180/math.Pi)
		row[11] = fmt.Sprintf("%.3f", alpha*180/math.Pi)
		row[12] = fmt.Sprintf("%.3f", phi*180/math.Pi)

		for _, val := range row {
			fmt.Fprintf(file, "%s\t", val)
		}
		fmt.Fprintln(file)
	}
}

func CSVTrajOutput(result [][]na.Point2D, filename string, t_r, t_1, theta_1dot, theta_2 float64) {
	file, err := os.Create(filename)
	if err != nil {
		panic(err)
	}
	defer file.Close()

	writer := csv.NewWriter(file)
	writer.Comma = ';'
	defer writer.Flush()

	writer.Write([]string{"t, с", "m, кг", "V_x, м/с", "V_y, м/с", "x, м", "y, м", "h, м", "V, м/с", "theta, град.", "Theta, град.", "Theta_с, град.", "alpha, град.", "phi, град."})
	// writer.Write([]string{"t", "Vx", "Vy", "X", "Y", "m", "h", "abs(R)", "abs(V)", "Theta"})

	for i, _ := range result[0] {
		row := make([]string, 13)

		// t
		t := result[0][i].X
		row[0] = fmt.Sprintf("%.8f", t)
		for j, col := range result {
			row[j+1] = fmt.Sprintf("%.8f", col[i].Y)
		}

		// m, Vx, Vy, X, Y
		Vx := result[0][i].Y
		Vy := result[1][i].Y
		X := result[2][i].Y
		Y := result[3][i].Y + R_M
		m := result[4][i].Y

		row[1] = fmt.Sprintf("%.8f", m)
		row[2] = fmt.Sprintf("%.8f", Vx)
		row[3] = fmt.Sprintf("%.8f", Vy)
		row[4] = fmt.Sprintf("%.8f", X)
		row[5] = fmt.Sprintf("%.8f", Y)

		// h, V, theta, Theta, Theta_s, alpha, phi
		R := math.Sqrt(X*X + Y*Y)
		h := R - R_M
		V := math.Sqrt(Vx*Vx + Vy*Vy)
		theta := theta(false, t, t_r, t_1, theta_1dot, theta_2)
		Theta := math.Asin((Vx*X + Vy*Y) / (V * R)) // to current horizon
		Theta_s := math.Atan(Vy / Vx)               // to starting horizon
		alpha := theta - Theta_s
		phi := math.Asin(X / R)

		row[6] = fmt.Sprintf("%.8f", h)
		row[7] = fmt.Sprintf("%.8f", V)
		row[8] = fmt.Sprintf("%.8f", theta*180/math.Pi)
		row[9] = fmt.Sprintf("%.8f", Theta*180/math.Pi)
		row[10] = fmt.Sprintf("%.8f", Theta_s*180/math.Pi)
		row[11] = fmt.Sprintf("%.8f", alpha*180/math.Pi)
		row[12] = fmt.Sprintf("%.8f", phi*180/math.Pi)

		err := writer.Write(row)
		if err != nil {
			panic(err)
		}
	}

}

// see model.go
func theta(fromRight bool, t, t_r, t_1, theta_1dot, theta_2 float64) float64 {
	if (t < t_r) || (t == t_r && !fromRight) {
		return math.Pi / 2
	} else if (t == t_r && fromRight) || (t <= t_1) {
		return math.Pi/2 + theta_1dot*(t-t_r)
	} else {
		return theta_2
	}
}
