package traj

import (
	"math"
	"traj/atmosphere"
)

const (
	Rz   = 6371000.0 // Earth radius [m]
	MuZ  = 3.986e14  // Earth gravitational parameter [m^3/s^2]
	G0   = 9.80665   // standard gravity [m/s^2]
	P0   = 101325.0  // sea-level pressure [Pa]
	Hatm = atmosphere.HBoundary
)

const (
	RrefAll = 0.785  // full-rocket max radius [m] -> Aref = π·0.785² = 1.9359 m²
	Lref    = 17.823 // full-rocket length [m], nose tip to aft plane
)

// Aref is the reference area for aerodynamic forces/moments [m^2].
var Aref = math.Pi * RrefAll * RrefAll
