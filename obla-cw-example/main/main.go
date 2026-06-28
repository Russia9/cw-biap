package main

import (
	"fmt"
	"math"
	"obla"
	"os"
	"path/filepath"
)

func main() {
	// Model params
	m_0 := 3000. // [kg]
	W := 3440.   // [m/s]
	t_r := 14.0  // s

	P := []float64{10.07 * 1000, 8.42 * 1000} // [N]
	h := [][]float64{
		{165 * 1000, 308 * 1000},
		{165 * 1000, 201 * 1000},
	} // [m]
	t1 := [][]float64{
		{374.689, 448.308},
		{496.410, 531.017},
	}
	t2 := [][]float64{
		{718.361, 1357.341},
		{861.370, 1101.365},
	}

	// Additional params
	m_k := 615. // [kg]

	fmt.Println("P	theta_1dot	theta_2	t_1	t_2")
	for i, P_i := range P {
		for j, h_ij := range h[i] {
			t_max := W / P_i * (m_0 - m_k)
			t_1 := t1[i][j]
			t_2 := t2[i][j]

			// Numeric methods' params
			// RK4
			epsIntegr := 1e-9
			hIntegr := 0.1

			// Newton method
			u0 := []float64{-math.Pi / (t_max - t_r), -math.Pi / 6}
			deltaU := []float64{1e-9, 1e-9}
			epsNewt := 1e-8

			// Damped Newton method
			// t0 := []float64{-0.106383*P_i + 1372.34043, -0.132979*P_i + 1940.42553}
			// deltaT := []float64{1e-3, 1e-3}
			// epsOpt := 1e-5
			// alpha_0 := 0.001
			// C1 := 0.5

			theta_1dot, theta_2, tr := obla.CalculateNewton(m_0, W, P_i, t_r, t_1, t_2, h_ij, u0, deltaU, epsNewt, epsIntegr, hIntegr)
			fmt.Printf("%.6f	%.6f	%.6f	%.6f	%.6f	%.6f\n", P_i, h_ij, theta_1dot, theta_2, t_1, t_2)

			// Diffs
			// dm_pg/dm_k
			DeltaM := 1e-3
			m_1 := m_0 + DeltaM
			_, _, tr1 := obla.CalculateNewton(m_1, W, P_i, t_r, t_1, t_2, h_ij, u0, deltaU, epsNewt, epsIntegr, hIntegr)
			dif1 := (tr1[4][len(tr1[4])-1].Y - tr[4][len(tr[4])-1].Y - DeltaM) / DeltaM

			// dm_pg/dW
			DeltaW := 1e-3
			W_1 := W + DeltaW
			_, _, tr2 := obla.CalculateNewton(m_0, W_1, P_i, t_r, t_1, t_2, h_ij, u0, deltaU, epsNewt, epsIntegr, hIntegr)
			dif2 := (tr2[4][len(tr2[4])-1].Y - tr[4][len(tr[4])-1].Y) / DeltaW

			// Output
			dir := fmt.Sprintf("out/%.2f_%.0f", P_i/1000, h_ij/1000)
			os.MkdirAll(dir, os.ModePerm)
			obla.CSVTrajOutput(tr, filepath.Join(dir, "traj.csv"), t_r, t_1, theta_1dot, theta_2)
			obla.TrajParamsOutput(filepath.Join(dir, "params.txt"), theta_1dot, theta_2, t_1, t_2, dif1, dif2)
			obla.TypstTrajOutput(tr, filepath.Join(dir, "traj_out.csv"), t_r, t_1, t_2, theta_1dot, theta_2)
		}
	}
}
