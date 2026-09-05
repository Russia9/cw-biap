// rocket.scad — 2-stage rocket + 3rd-stage/bus, from the A(1:25) assembly drawing
//
// BRANCH NOTE (aero-anna): this is NOT the coursework vehicle. main.py does not
// own these dimensions and rocket-params.scad is not included — every number
// below is read straight off the drawing. Do not merge this file to main.
//
// The OUTER MOLD LINE only — the surface the flow sees. This is a CFD input,
// not an engineering drawing: no motor internals, no bores, no charge cavities
// and no nozzles. Every section is a solid of revolution, unioned with a small
// overlap, so each exported STL is one closed manifold shell.
//
// Staging: each interstage shroud belongs to the LOWER stage and drops with
// it — the 850 mm band with stage 1, the 1500 mm flare cone with stage 2.
//
// Unit = METERS. OpenSCAD is unitless; set SCALE=1000 for a millimetre STL.
//
// Preview:  open in OpenSCAD, F5 = preview, F6 = render.
// Export :  see Makefile — `make stls` writes one STL per flight configuration
//           (whole rocket / stage 2 & up / stage 3 & up / head) for aero analysis.
//           e.g.  openscad -D 'PART="stage2up"' -o stage2up.stl rocket.scad

$fn = 360;

// ---- Export selection (override via -D on the CLI) ----
PART  = "all";   // "all" | "stage2up" | "stage3up" | "head"
SCALE = 1;       // 1 = metres (model units); 1000 = millimetres

eps = 0.003;     // small overlap so stacked sections fuse into one manifold solid

// ============================================================================
//  Drawing dimensions — axial chain from the nose tip, total 31610 mm.
//
//    0 ..  3000   nose cone (pos. 2)                      -> d_fwd
//    3000 ..  4200   cylinder  (pos. 3)          1200 |
//    4200 ..  5700   cylinder  (pos. 4)          1500 |  d_fwd
//    5700 ..  6980   3rd-stage motor (pos. 5, 6) 1280 |
//    6980 ..  7880   its nozzle bay (pos. 7)      900 |
//    7880 ..  9380   interstage flare            1500     d_fwd -> d_body
//    9380 .. 20130   2nd-stage case (pos. 9)    10750 |
//   20130 .. 20980   interstage band             850 |
//   20980 .. 28510   1st-stage case (pos. 10)   7530 |    d_body
//   28510 .. 31610   aft skirt / nozzle bay     3100 |
//
//  The 850 mm band is the one length not called out on the drawing; it is the
//  balance of the 31610 total against the nine dimensioned segments, and it
//  matches the double vertical line drawn between the two cases.
//
//  The aft 3100 mm carries the stage-1 nozzle INSIDE a skirt at d_body — the
//  measured silhouette runs straight to the tail with no boat-tail — so
//  ignoring the nozzle costs the mold line nothing: it stays a cylinder.
//
//  Section A(1:25) gives d_body = 2440 and d_fwd = 1450. Its third circle,
//  Ø380, is the stage-1 throat; not modelled.
// ============================================================================
d_body = 2.440;   // both motor cases, the interstage band and the aft skirt
d_fwd  = 1.450;   // everything forward of the interstage flare

L_nose     =  3.000;   // pos. 2, a plain cone (the drawn profile is straight)
L_head     =  2.700;   // pos. 3 + pos. 4          = 1200 + 1500
L_stage3   =  2.180;   // pos. 5, 6 + pos. 7       = 1280 +  900
L_flare    =  1.500;   // interstage cone d_fwd -> d_body
L_case2    = 10.750;   // pos. 9
L_band     =  0.850;   // undimensioned interstage band, drops with stage 1
L_case1    =  7.530;   // pos. 10
L_skirt    =  3.100;   // aft skirt enclosing the stage-1 nozzle

// Colours (preview only; STL has no colour).
C_S1  = [0.70, 0.70, 0.72];
C_S2  = [0.76, 0.76, 0.78];
C_S3  = [0.82, 0.82, 0.84];
C_PL  = [0.80, 0.30, 0.30];   // head
C_STR = [0.45, 0.45, 0.48];   // structure: bands, flare, nose

// ---------- primitive helpers ----------
module tube(d, h)             cylinder(d = d, h = h);
module frustum(d_lo, d_hi, h) cylinder(d1 = d_lo, d2 = d_hi, h = h);
module cone_tip(d, h)         cylinder(d1 = d, d2 = 0, h = h);

// stacked segments with an eps overlap into their neighbours (manifold STL)
module seg_tube(z, h, d)          translate([0, 0, z - eps]) tube(d, h + 2 * eps);
module seg_cone(z, h, d_lo, d_hi) translate([0, 0, z]) frustum(d_lo, d_hi, h);

// ---------- cumulative z (aft end of stage 1 at z = 0, nose at z_top) ----------
z_skirt  = 0;
z_case1  = z_skirt  + L_skirt;
z_band   = z_case1  + L_case1;
z_case2  = z_band   + L_band;     // stage 1/2 separation plane
z_flare  = z_case2  + L_case2;
z_stage3 = z_flare  + L_flare;    // stage 2/3 separation plane
z_head   = z_stage3 + L_stage3;   // head separation plane
z_nose   = z_head   + L_head;
z_top    = z_nose   + L_nose;

// ---------- section groups (defined in global coordinates) ----------
// Each lower stage carries the interstage above it, which drops with it.

module stage1_group() {
    color(C_S1)  seg_tube(z_skirt, L_skirt + L_case1, d_body);  // skirt + case
    color(C_STR) seg_tube(z_band, L_band, d_body);              // interstage band
}

module stage2_group() {
    color(C_S2)  seg_tube(z_case2, L_case2, d_body);
    color(C_STR) seg_cone(z_flare, L_flare, d_body, d_fwd);     // interstage flare
}

module stage3_group() {
    color(C_S3) seg_tube(z_stage3, L_stage3, d_fwd);
}

module head_group() {
    color(C_PL)  seg_tube(z_head, L_head, d_fwd);
    color(C_STR) translate([0, 0, z_nose]) cone_tip(d_fwd, L_nose);
}

// ---------- configuration assembly ----------
module assembly(part) {
    if (part == "all") stage1_group();
    if (part == "all" || part == "stage2up") stage2_group();
    if (part == "all" || part == "stage2up" || part == "stage3up") stage3_group();
    head_group();   // present in every configuration
}

// CFD orientation (arc-case convention): lay the rocket axis on +x with the nose
// tip at x = 0 pointing upstream (-x), so the freestream travels +x and meets the
// nose first. translate(-z_top) drops the nose tip (local z_top, present in every
// configuration) onto the origin; rotate([0,-90,0]) then maps the +z build axis
// onto +x, placing the aft-most plane at +x = L_model. Units stay in metres
// (SCALE = 1), so the OpenFOAM blockMesh/snappy read it with scale 1.
scale(SCALE) rotate([0, -90, 0]) translate([0, 0, -z_top]) assembly(PART);

// ---------- verification echo (Console window) ----------
echo(str("PART=", PART, "  SCALE=", SCALE));
echo(str("Diameters: body d=", d_body, " m   forward d=", d_fwd, " m"));
echo(str("Separation planes from the nose tip: 1/2 at ", z_top - z_case2,
         " m,  2/3 at ", z_top - z_stage3, " m,  head at ", z_top - z_head, " m"));
echo(str("Model lengths: all=", z_top, "  stage2up=", z_top - z_case2,
         "  stage3up=", z_top - z_stage3, "  head=", z_top - z_head, " m"));
