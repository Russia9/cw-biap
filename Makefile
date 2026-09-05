# Generate per-configuration STL meshes from rocket.scad for aerodynamic analysis.
#
#   make stls             # all four meshes (model units = metres)
#   make stls SCALE=1000  # millimetre meshes
#   make head.stl         # a single configuration
#   make params           # regenerate rocket-params.scad from main.py
#   make png              # quick preview render
#   make clean
#
# SCALE is not tracked by make, so switching it does not force a rebuild; run
# `make clean` first. openfoam/gen_case.py always builds with SCALE=1.

OPENSCAD ?= openscad
UV       ?= uv
SCALE    ?= 1
SCAD     := rocket.scad
PARAMS   := rocket-params.scad
STLS     := rocket.stl stage2up.stl stage3up.stl head.stl

# main.py reads utils.py, typst.py and the digitized chart CSVs, so all of them
# feed the generated parameter block.
PARAM_DEPS := main.py utils.py typst.py $(wildcard assets/*.csv)

.PHONY: stls params png clean
stls: $(STLS)
params: $(PARAMS)

# map each output mesh to its PART selector
rocket.stl:   PART := all
stage2up.stl: PART := stage2up
stage3up.stl: PART := stage3up
head.stl:     PART := head

# main.py prints the whole Typst report on stdout; only the file it writes and
# its stderr status line matter here.
$(PARAMS): $(PARAM_DEPS)
	$(UV) run python main.py --write-scad-params >/dev/null

# rocket.scad hardcodes this branch's dimensions and no longer includes
# $(PARAMS), so the STLs do not depend on main.py. `make params` still works.
$(STLS): %.stl: $(SCAD)
	$(OPENSCAD) -D 'PART="$(PART)"' -D 'SCALE=$(SCALE)' -o $@ $<

# --autocenter --viewall frames the whole stack; without them OpenSCAD's default
# camera lands mid-body and the render shows a featureless tube.
png: $(SCAD)
	$(OPENSCAD) -D 'PART="all"' --autocenter --viewall --camera=0,0,0,68,0,20,0 \
	    --imgsize=1400,500 -o rocket.png $(SCAD)

# rocket-params.scad is tracked and regenerable, but removing it would break a
# bare OpenSCAD open, so clean leaves it alone.
clean:
	rm -f $(STLS) rocket.png
