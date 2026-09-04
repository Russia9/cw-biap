package traj

import "math"

// eq determines if x and y are equal with eps precision
func eq(x, y float64) bool {
	const eps = 1e-10
	return math.Abs(x-y) <= eps
}
