package traj

import "math"

// Physical constants (central-gravity, non-rotating spherical Earth — §4.1/§4.2).
const (
	Rz   = 6371000.0 // Earth radius [m]
	MuZ  = 3.986e14  // Earth gravitational parameter [m^3/s^2]
	G0   = 9.80665   // standard gravity [m/s^2]
	P0   = 101325.0  // sea-level pressure [Pa]
	Hatm = 94000.0   // conditional atmosphere boundary [m]
)

const (
	// Reference geometry shared by all aerodynamic coefficients, with the
	// centre of rotation at the nose. Both numbers are the bounding box of
	// rocket.stl (PART="all") as openfoam/gen_case.py's stl_bbox() measures it.
	// That is by construction the reference the CFD non-dimensionalised by:
	// gen_case.py writes Aref = π·R_all², lRef = L_all and CofR = (0 0 0) into
	// each case's constant/freestreamProperties from that same bounding box.
	//
	// Refresh after any change to main.py's d_(м i)/L_i or to rocket.scad:
	//
	//	make rocket.stl
	//	uv run python -c "import sys; from pathlib import Path; \
	//	  sys.path.insert(0,'openfoam'); from gen_case import stl_bbox; \
	//	  print(stl_bbox(Path('rocket.stl')))"
	//
	// L_all is the 18.24 m stack height (sum L_i = 14.85 plus 3.39 m of
	// payload and interstage section) plus the 3 mm eps overhang rocket.scad
	// uses to fuse stacked sections into one solid. R_all is d_ext[0]/2 =
	// d_(м 1)/2 = 1.58/2 — stage 1 sets the maximum diameter.
	RrefAll = 0.79   // full-rocket max radius [m] -> Aref = π·0.79² = 1.9607 m²
	Lref    = 18.243 // full-rocket length [m], nose tip to aft plane
)

// Aref is the reference area for aerodynamic forces/moments [m²].
var Aref = math.Pi * RrefAll * RrefAll

// DragScale multiplies the tabulated drag coefficient, for sensitivity studies
// against the CFD data. Set to 0.9 to use 90 % of the tabulated Cd. Has no
// effect while aerodynamics are zeroed (see ZeroAero in aero.go).
const DragScale = 1.0

// Angle conversions shared across the package.
const r2d = 180 / math.Pi

func deg(d float64) float64 { return d * math.Pi / 180 }
