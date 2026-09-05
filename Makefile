# Geometry pipeline: report/main.py -> openscad/ -> STL -> openfoam/input/.
#
#   make stls             # all four meshes into openscad/out/ (model units = metres)
#   make stls SCALE=1000  # millimetre meshes
#   make openscad/out/head.stl   # a single configuration
#   make openfoam-input   # stage the meshes into openfoam/input/ for the CFD chain
#   make params           # regenerate openscad/rocket-params.scad from report/main.py
#   make png              # quick preview render
#   make clean
#
# SCALE is not tracked by make, so switching it does not force a rebuild; run
# `make clean` first. openfoam/gen_case.py always consumes SCALE=1 meshes.

OPENSCAD ?= openscad
UV       ?= uv
SCALE    ?= 1

SCAD     := openscad/rocket.scad
PARAMS   := openscad/rocket-params.scad
OUT      := openscad/out
FOAM_IN  := openfoam/input

PARTS    := all stage2up stage3up head
STLS     := $(addprefix $(OUT)/,$(addsuffix .stl,$(PARTS)))
FOAM_STLS:= $(addprefix $(FOAM_IN)/,$(addsuffix .stl,$(PARTS)))

# main.py reads utils.py, typst.py and the digitized chart CSVs, so all of them
# feed the generated parameter block.
PARAM_DEPS := report/main.py report/utils.py report/typst.py $(wildcard report/assets/*.csv)

.PHONY: stls params png openfoam-input clean
stls: $(STLS)
params: $(PARAMS)

# main.py prints the whole Typst report on stdout; only the file it writes and
# its stderr status line matter here.
$(PARAMS): $(PARAM_DEPS)
	$(UV) run python report/main.py --write-scad-params >/dev/null

# The stem is the PART selector, so `all.stl` renders PART="all". $(SCAD) MUST
# stay the first prerequisite: the recipe passes $< to OpenSCAD as the input file.
$(OUT)/%.stl: $(SCAD) $(PARAMS) | $(OUT)
	$(OPENSCAD) -D 'PART="$*"' -D 'SCALE=$(SCALE)' -o $@ $<

# The CFD module reads its geometry from openfoam/input/ and never builds it,
# so this is the one hand-off between the two modules.
openfoam-input: $(FOAM_STLS)

$(FOAM_IN)/%.stl: $(OUT)/%.stl | $(FOAM_IN)
	cp $< $@

$(OUT) $(FOAM_IN):
	mkdir -p $@

# --autocenter --viewall frames the whole stack; without them OpenSCAD's default
# camera lands mid-body and the render shows a featureless tube.
png: $(SCAD) $(PARAMS) | $(OUT)
	$(OPENSCAD) -D 'PART="all"' --autocenter --viewall --camera=0,0,0,68,0,20,0 \
	    --imgsize=1400,500 -o $(OUT)/rocket.png $(SCAD)

# rocket-params.scad is tracked and regenerable, but removing it would break a
# bare OpenSCAD open, so clean leaves it alone.
clean:
	rm -rf $(OUT) $(FOAM_IN)
