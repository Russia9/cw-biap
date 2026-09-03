package traj

import (
	"math"

	"traj/atmosphere"
)

// Physical constants (central-gravity, non-rotating spherical Earth — §4.1/§4.2).
//
// Two deliberate convention splits, flagged in the physics audit:
//   - Rz is the mean Earth radius used by the dynamics; the atmosphere package
//     internally uses the GOST geopotential radius 6 356 767 m. The 14 km gap
//     only enters the geopotential conversion (sub-metre effect on H').
//   - G0 appears solely in the Isp→thrust conversion, because specific impulse
//     in seconds is defined against standard gravity. Surface gravity in the
//     model itself is MuZ/Rz² ≈ 9.820 m/s².
const (
	Rz  = 6371000.0 // Earth radius [m]
	MuZ = 3.986e14  // Earth gravitational parameter [m^3/s^2]
	G0  = 9.80665   // standard gravity [m/s^2]
	P0  = 101325.0  // sea-level pressure [Pa]
	// Hatm is the conditional atmosphere boundary [m], above which the
	// atmosphere model returns vacuum — the same boundary the §4.4 crossing
	// diagnostics report against.
	Hatm = atmosphere.HBoundary
	// QSepMin is the dynamic pressure [Pa] above which a stage separation
	// counts as happening "inside the atmosphere" for the |α| separation
	// limit. The gate is on q rather than on H because separation loads are a
	// dynamic-pressure phenomenon: an H ≤ Hatm gate would sit within metres of
	// the 2/3 separation (H = 92.8…96.7 km across nearby designs), flipping the
	// constraint on and off with sub-second timing changes, and it would reward
	// lofting that separation above 94 km purely to escape the limit. At 1 kPa
	// the stage-1 separation (q ≈ 13 kPa) is 13× inside the gate and the stage-2
	// one (q ≤ 21 Pa) is far outside it, so the classification is stable.
	QSepMin = 1000.0
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
	// L_all is the 16.39 m stack height (sum L_i = 12.39 plus interstage and
	// payload sections, plus the 3 mm eps overhang rocket.scad uses to fuse
	// stacked sections into one solid). R_all is d_ext[0]/2 = d_(м 1)/2 =
	// 1.57/2 — stage 1 sets the maximum diameter.
	//
	// Lref shrank from 17.823 when (3.44) dropped l_дк and l_в from L_i; RrefAll
	// did not move, because L_i does not reach any diameter. That asymmetry is
	// what makes openfoam/results/averages.csv still usable — see the note on
	// its staleness in CLAUDE.md.
	RrefAll = 0.785  // full-rocket max radius [m] -> Aref = π·0.785² = 1.9359 m²
	Lref    = 16.393 // full-rocket length [m], nose tip to aft plane
)

// Aref is the reference area for aerodynamic forces/moments [m²].
var Aref = math.Pi * RrefAll * RrefAll

// Angle conversions shared across the package.
const r2d = 180 / math.Pi

func deg(d float64) float64 { return d * math.Pi / 180 }
