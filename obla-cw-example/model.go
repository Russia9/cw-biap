package obla

import (
	"math"

	na "github.com/Russia9/numerical-analysis"
)

const (
	R_M = 1738.0 * 1000 // Moon radius [m]

	mu_M = 4.903 * 10e11 // Moon gravitational parameter [km^3/s^2]
)

// g_x = -(mu_m*x)/( (x^2+(R_m*y)^2)^(3/2) )
func g_x(_ bool, _ float64, y ...float64) float64 {
	return -1 * (mu_M * y[2]) / (math.Pow(y[2]*y[2]+(R_M+y[3])*(R_M+y[3]), 1.5))
}

// g_y = -(mu_m*(R_M + y))/( (x^2+(R_m*y)^2)^(3/2) )
func g_y(_ bool, _ float64, y ...float64) float64 {
	return -1 * (mu_M * (R_M + y[3])) / (math.Pow(y[2]*y[2]+(R_M+y[3])*(R_M+y[3]), 1.5))
}

// m_0 - Initial mass [kg]
// W_src - Effective fuel velocity [m/s]
// P - Thrust [N]
// t_r - Rotation time [s]
// t_1 - Engine shutdown time [s]
// t_2 - Engine second turn on time [s]
// ---
// model - DE system for specified parameters
// tChar - Characteristic times [s]
func InitModel(m_0, W_src, Pbase, t_r, t_1, t_2, theta_1dot, theta_2 float64) (model na.FuncSystem) {
	beta := func(fromRight bool, x float64, _ ...float64) float64 {
		// Since between t_1 and t_2 the engine is turned off, the mass decreases when t in [0,t_1] + [t_2, +inf]
		if (x < t_1) || (x == t_1 && !fromRight) {
			return Pbase / W_src
		} else if (x < t_2) || (x == t_1 && fromRight) || (x == t_2 && !fromRight) {
			return 0
		} else {
			return Pbase / W_src
		}
	} // Fuel consumption rate [kg/s]

	P := func(fromRight bool, x float64, _ ...float64) float64 {
		return W_src * beta(fromRight, x)
	}

	theta := func(fromRight bool, x float64, _ ...float64) float64 {
		if (x < t_r) || (x == t_r && !fromRight) {
			return math.Pi / 2
		} else if (x == t_r && fromRight) || (x <= t_1) {
			return math.Pi/2 + theta_1dot*(x-t_r)
		} else {
			return theta_2
		}
	}

	return na.FuncSystem{
		// model[0] = dVx/dt
		func(fromRight bool, x float64, y ...float64) float64 {
			return P(fromRight, x, y...)/y[4]*math.Cos(theta(fromRight, x, y...)) + g_x(fromRight, x, y...)
		},
		// model[1] = dVy/dt
		func(fromRight bool, x float64, y ...float64) float64 {
			return P(fromRight, x, y...)/y[4]*math.Sin(theta(fromRight, x, y...)) + g_y(fromRight, x, y...)
		},
		// model[2] = dx/dt
		func(fromRight bool, x float64, y ...float64) float64 {
			return y[0]
		},
		// model[3] = dy/dt
		func(fromRight bool, x float64, y ...float64) float64 {
			return y[1]
		},
		// model[4] = dm/dt
		func(fromRight bool, x float64, y ...float64) float64 {
			return -beta(fromRight, x, y...)
		},
	}
}

func CalculateTrajectory(m_0, W_src, P, t_r, t_1, t_2, theta_1dot, theta_2, h_ams, eps, h float64) [][]na.Point2D {
	model := InitModel(m_0, W_src, P, t_r, t_1, t_2, theta_1dot, theta_2)
	tChar := []float64{t_r, t_1, t_2}
	start := []float64{0, 0, 0, 0, m_0}

	R_ams := R_M + h_ams
	V_ams := math.Sqrt(mu_M / R_ams)

	stop := func(x float64, y ...float64) (bool, bool) {
		V := math.Sqrt(y[0]*y[0] + y[1]*y[1])
		return V >= V_ams, math.Abs(V-V_ams) < eps
	}

	result, err := na.RungeKuttaMethod(model, 0, start, tChar, h, stop)
	if err != nil {
		panic(err)
	}

	return result
}

func CalculateNewton(m_0, W_src, P, t_r, t_1, t_2, h_ams float64, u0, deltaU []float64, epsNewt, epsIntegr, h float64) (theta_1dot, theta_2 float64, tr [][]na.Point2D) {
	// u0[1] = theta_1dot
	// u0[2] = theta_2
	// F1 = Delta r
	// F2 = Delta theta
	res, err := na.SENewton([]func(u []float64) float64{
		func(u []float64) float64 {
			tr := CalculateTrajectory(m_0, W_src, P, t_r, t_1, t_2, u[0], u[1], h_ams, epsIntegr, h)
			X := tr[2][len(tr[2])-1].Y
			Y := tr[3][len(tr[3])-1].Y + R_M
			return math.Sqrt(X*X+Y*Y) - (h_ams + R_M)
		},
		func(u []float64) float64 {
			tr := CalculateTrajectory(m_0, W_src, P, t_r, t_1, t_2, u[0], u[1], h_ams, epsIntegr, h)

			V_x := tr[0][len(tr[0])-1].Y
			V_y := tr[1][len(tr[1])-1].Y
			V := math.Sqrt(V_x*V_x + V_y*V_y)

			X := tr[2][len(tr[2])-1].Y
			Y := tr[3][len(tr[3])-1].Y + R_M
			R := math.Sqrt(X*X + Y*Y)

			return math.Asin((V_x*X + V_y*Y) / (V * R))
		},
	}, u0, deltaU, epsNewt)
	if err != nil {
		panic(err)
	}

	theta_1dot = res[0]
	theta_2 = res[1]

	tr = CalculateTrajectory(m_0, W_src, P, t_r, t_1, t_2, theta_1dot, theta_2, h_ams, epsIntegr, h)

	return theta_1dot, theta_2, tr
}

func CalculateOptimizaiton(m_0, W_src, P, t_r, h_ams float64, u0, deltaU []float64, t0, deltaT []float64, alpha0, C1, epsOpt, epsNewt, epsIntegr, h float64) (theta1dot, theta2, t1, t2 float64, tr [][]na.Point2D) {
	res, err := na.DampedNewtonExtremum(func(x []float64) float64 {
		_, _, tr := CalculateNewton(m_0, W_src, P, t_r, x[0], x[1], h_ams, u0, deltaU, epsNewt, epsIntegr, h)
		return m_0 - tr[4][len(tr[4])-1].Y
	}, t0, deltaT, alpha0, C1, epsOpt, 10)
	if err != nil {
		panic(err)
	}

	t1 = res[0]
	t2 = res[1]

	theta1dot, theta2, tr = CalculateNewton(m_0, W_src, P, t_r, t1, t2, h_ams, u0, deltaU, epsNewt, epsIntegr, h)

	return theta1dot, theta2, t1, t2, tr
}
