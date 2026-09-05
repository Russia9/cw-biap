package traj

import "math"

// Eq determines if x and y are equal with eps precision
func Eq(x, y, eps float64) bool {
	return math.Abs(x-y) <= eps
}
