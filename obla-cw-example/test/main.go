package main

import (
	"fmt"
	"math"
	"obla"
)

func main() {
	// Model params
	m_0 := 3000. // [kg]
	W := 3420.   // [m/s]
	t_r := 14.0  // s

	P := []float64{10.08 * 1000, 8.20 * 1000} // [N]
	h := [][]float64{
		{160 * 1000, 269 * 1000},
		{160 * 1000, 210 * 1000},
	} // [m]
	t1 := [][]float64{
		{369.818258, 432.764059},
		{513.293946, 567.602972},
	}
	t2 := [][]float64{
		{698.934917, 1158.537607},
		{867.204537, 1335.400719},
	}

	// Additional params
	m_k := 615. // [kg]

	for i, P_i := range P {
		for j, h_ij := range h[i] {
			t_max := W / P_i * (m_0 - m_k)

			// Numeric methods' params
			// RK4
			epsIntegr := 1e-9
			hIntegr := 0.1

			// Newton method
			u0 := []float64{-math.Pi / (t_max - t_r), -math.Pi / 6}
			deltaU := []float64{1e-9, 1e-9}
			epsNewt := 1e-8

			fmt.Printf("P=%.7f, theta_1dot=%.7f, theta_2=%.7f, t_1=%.7f, t_2=%.7f h=%.7f\n", P_i, u0[0], u0[1], t1[i][j], t2[i][j], h_ij)

			// Grid calculation
			N := 12
			epsilon_t1 := 20. // area to distribute N points around t1[i][j]
			epsilon_t2 := 20. // area to distribute N points around t2[i][j]

			t1_grid := make([]float64, N)
			t2_grid := make([]float64, N)
			deltam_grid := make([][]float64, N)

			// Generate t1 and t2 grids
			for k := range N {
				t1_grid[k] = t1[i][j] + (float64(k)-float64(N-1)/2)*epsilon_t1/float64(N-1)
				t2_grid[k] = t2[i][j] + (float64(k)-float64(N-1)/2)*epsilon_t2/float64(N-1)
			}

			// Calculate deltam for each grid point
			for k := range N {
				deltam_grid[k] = make([]float64, N)
				for l := range N {
					_, _, tr := obla.CalculateNewton(m_0, W, P_i, t_r, t1_grid[k], t2_grid[l], h_ij, u0, deltaU, epsNewt, epsIntegr, hIntegr)
					deltam_grid[k][l] = m_0 - tr[4][len(tr[4])-1].Y
				}
			}

			// Output in requested format
			fmt.Print("(")
			for k, val := range t1_grid {
				if k > 0 {
					fmt.Print(", ")
				}
				fmt.Printf("%.7f", val)
			}
			fmt.Println("),")

			fmt.Print("(")
			for k, val := range t2_grid {
				if k > 0 {
					fmt.Print(", ")
				}
				fmt.Printf("%.7f", val)
			}
			fmt.Println("),")

			fmt.Println("(")
			for k := range N {
				fmt.Print(" (")
				for l, val := range deltam_grid[k] {
					if l > 0 {
						fmt.Print(", ")
					}
					fmt.Printf("%.7f", val)
				}
				fmt.Println("),")
			}
			fmt.Println("),")

			// Central point
			fmt.Println("Central point:")
			fmt.Printf("(%.7f,),(%.7f,),\n", t1[i][j], t2[i][j])
			fmt.Println()

		}
	}
}
