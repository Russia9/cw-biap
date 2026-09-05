package traj

import "math"

// Eq determines if x and y are equal with eps precision
func Eq(x, y float64) bool {
	const eps = 1e-10
	return math.Abs(x-y) <= eps
}
