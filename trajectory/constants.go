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
	Lref    = 16.393 // full-rocket length [m], nose tip to aft plane
	// Both are the all.stl bounding box from openfoam/gen_case.py's stl_bbox(),
	// the same function that writes Aref/lRef into every CFD case. Refresh after any
	// change to report/main.py's d_(м i)/L_i or to openscad/rocket.scad:
	//   make stls && uv run python -c "import sys; sys.path.insert(0,'openfoam');
	//   from pathlib import Path; from gen_case import stl_bbox;
	//   print(stl_bbox(Path('openscad/out/all.stl')))"
)

// Aref is the reference area for aerodynamic forces/moments [m^2].
var Aref = math.Pi * RrefAll * RrefAll

const r2d = 180 / math.Pi
const d2r = math.Pi / 180
