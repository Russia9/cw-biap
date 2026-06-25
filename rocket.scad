// rocket.scad — 3-stage solid-fuel rocket (BIAP coursework)
//
// Mimics the prototype ICBM silhouette (pointed nose, three stage cylinders of
// decreasing diameter, interstage shrouds, 4 nozzles per stage) using THIS
// rocket's own dimensions.
//
// Staging: each interstage skirt belongs to the LOWER stage and encloses the
// upper stage's nozzles; it drops together with that lower stage, exposing the
// next stage's nozzles. The first stage has no skirt (its nozzles are bare).
//
// Stage 3 has the same motor-case diameter as stage 2 (d_м3 = d_м2 = 1.16 m),
// so the stages mate flush with no interstage skirt; stage 3's nozzles hang bare
// below its body (same as stage 1).
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
//  Calculated parameters — straight from main.py (index 0/1/2 = stage 1/2/3).
//  These mirror the report tables; verify against the script output, do not
//  tune them here.
// ============================================================================
d_m  = [1.58,  1.17,  1.17 ];  // motor-case diameter  d_(м i)    (table 3.13); d_m[2]=d_m[1] by design
L    = [7.72,  4.33,  2.85 ];  // overall stage length L_i        (table 3.14)
d_kr = [0.168, 0.135, 0.086];  // nozzle throat dia    d_("кр" i)  (table 3.14)
d_a  = [0.452, 0.466, 0.447];  // nozzle exit dia      d_(a i)     (table 3.14)
l_a  = [0.390, 0.454, 0.495];  // nozzle bell length   l_(a i)     (divergent cone)

n_noz   = 4;                   // nozzles per stage
noz_gap = [1.3, 1.06, 1.06];     // per stage; >1 spreads the 4 bells apart (genus-0 mesh)

// ============================================================================
//  Structural / CAD-drawing parameters — chosen here, not produced by main.py.
// ============================================================================

// Interstage 1-2 shroud = skirt (d_m[0]) + cone; belongs to stage 1, encloses
// stage 2's nozzles, drops with stage 1.
H_sk12 = 0.40; H_cone21 = 0.40;   // stage 1/2 interstage (d_m[0] -> d_m[1])

// Navigation + head (read from the CAD drawing; edit freely).
d_ns = 0.70; L_ns = 0.18;   // navigation module
d_pl = 0.54; L_pl = 0.65;   // head / warhead body
h_nose = 0.51;              // warhead nose cone
H_s3nav   = 0.75;           // adapter stage 3 -> nav  (1.16 -> 0.70)
H_navhead = 0.5;           // adapter nav -> head     (0.70 -> 0.54)

// Colours (preview only; STL has no colour). C_S indexed by stage.
C_S  = [[0.70, 0.70, 0.72],   // stage 1
        [0.76, 0.76, 0.78],   // stage 2
        [0.82, 0.82, 0.84]];  // stage 3
C_NAV = [0.30, 0.50, 0.80];   // navigation block
C_PL  = [0.80, 0.30, 0.30];   // warhead
C_STR = [0.45, 0.45, 0.48];   // structure: skirts, cones, nose, nozzles

// ---------- primitive helpers ----------
module tube(d, h)             cylinder(d = d, h = h);
module frustum(d_lo, d_hi, h) cylinder(d1 = d_lo, d2 = d_hi, h = h);
module cone_tip(d, h)         cylinder(d1 = d, d2 = 0, h = h);

// stacked segments with an eps overlap into their neighbours (manifold STL)
module seg_tube(z, h, d)          translate([0, 0, z - eps]) tube(d, h + 2 * eps);
module seg_cone(z, h, d_lo, d_hi) translate([0, 0, z]) frustum(d_lo, d_hi, h);

// 4 after-critical nozzle bells hanging below the stage base z_top (throat at z_top)
module nozzles(z_top, st) {
    de = d_a[st]; dt = d_kr[st]; h = l_a[st];
    br = de / sqrt(2) * noz_gap[st];   // bolt circle: spread the 4 bells per stage
    for (i = [0 : n_noz - 1])
        rotate([0, 0, 45 + i * 360 / n_noz])
            translate([br, 0, z_top - h])
                cylinder(d1 = de, d2 = dt, h = h + eps);
}

// ---------- cumulative z (stage-1 base at z = 0) ----------
z_body1  = 0;
z_sk12   = z_body1 + L[0];     // interstage skirt 1-2 (d_m[0], encloses stage-2 nozzles)
z_cone21 = z_sk12 + H_sk12;    // cone d_m[0] -> d_m[1]
z_body2  = z_cone21 + H_cone21;
z_body3  = z_body2 + L[1];     // stages 2/3 mate flush (equal diameters, no skirt)
z_s3nav  = z_body3 + L[2];
z_nav    = z_s3nav + H_s3nav;
z_navhead = z_nav + L_ns;
z_head   = z_navhead + H_navhead;
z_nose   = z_head + L_pl;
z_top    = z_nose + h_nose;

// ---------- section groups (defined in global coordinates) ----------
// Stage 1 carries the interstage shroud above it that encloses stage 2's nozzles;
// dropping stage 1 exposes them. Stage 2 carries the interstage skirt that encloses
// stage 3's nozzles; dropping stage 2 exposes them.

module stage1_group() {
    color(C_STR) nozzles(z_body1, 0);              // bare (no skirt on stage 1)
    color(C_S[0]) seg_tube(z_body1, L[0], d_m[0]);
    color(C_STR) {
        seg_tube(z_sk12, H_sk12, d_m[0]);          // skirt enclosing stage-2 nozzles
        seg_cone(z_cone21, H_cone21, d_m[0], d_m[1]);
    }
}

module stage2_group() {
    color(C_STR) nozzles(z_body2, 1);              // exposed once stage 1 drops
    color(C_S[1]) seg_tube(z_body2, L[1], d_m[1]);
}

module stage3_group() {
    color(C_STR) nozzles(z_body3, 2);              // exposed once stage 2 drops
    color(C_S[2]) seg_tube(z_body3, L[2], d_m[2]); // motor case
}

module nav_group() {
    color(C_STR) seg_cone(z_s3nav, H_s3nav, d_m[2], d_ns);
    color(C_NAV) seg_tube(z_nav, L_ns, d_ns);
    color(C_STR) seg_cone(z_navhead, H_navhead, d_ns, d_pl);
}

module head_group() {
    color(C_PL)  seg_tube(z_head, L_pl, d_pl);
    color(C_STR) translate([0, 0, z_nose]) cone_tip(d_pl, h_nose);
}

// ---------- configuration assembly ----------
module assembly(part) {
    if (part == "all") stage1_group();
    if (part == "all" || part == "stage2up") stage2_group();
    if (part == "all" || part == "stage2up" || part == "stage3up") {
        stage3_group();
        nav_group();
    }
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
echo(str("Stage diameters d_m = ", d_m, " m"));
echo(str("Stage lengths   L   = ", L, " m"));

echo(str("Nav: d=", d_ns, " L=", L_ns, "   Head: d=", d_pl, " L=", L_pl, " + nose ", h_nose));
echo(str("Nozzles/stage=", n_noz, "  throat d_kr=", d_kr, " exit d_a=", d_a, " bell l_a=", l_a, " gap=", noz_gap));
echo(str("Full height (excl. nozzles) = ", z_top, " m"));
