# Generate per-configuration STL meshes from rocket.scad for aerodynamic analysis.
#
#   make stls             # all four meshes (model units = metres)
#   make stls SCALE=1000  # millimetre meshes
#   make head.stl         # a single configuration
#   make png              # quick preview render
#   make clean

OPENSCAD ?= openscad
SCALE    ?= 1
SCAD     := rocket.scad
STLS     := rocket.stl stage2up.stl stage3up.stl head.stl

.PHONY: stls png clean
stls: $(STLS)

# map each output mesh to its PART selector
rocket.stl:   PART := all
stage2up.stl: PART := stage2up
stage3up.stl: PART := stage3up
head.stl:     PART := head

$(STLS): %.stl: $(SCAD)
	$(OPENSCAD) -D 'PART="$(PART)"' -D 'SCALE=$(SCALE)' -o $@ $<

png: $(SCAD)
	$(OPENSCAD) -D 'PART="all"' --imgsize=600,1200 -o rocket.png $(SCAD)

clean:
	rm -f $(STLS) rocket.png
