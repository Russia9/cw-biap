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
// Unit = METERS. OpenSCAD is unitless; set SCALE=1000 for a millimetre STL.
//
// Preview:  open in OpenSCAD, F5 = preview, F6 = render.
// Export :  see Makefile — `make stls` writes one STL per flight configuration
//           (whole rocket / stage 2 & up / stage 3 & up / head) for aero analysis.
//           e.g.  openscad -D 'PART="stage2up"' -o stage2up.stl rocket.scad

$fn = 120;

// ---- Export selection (override via -D on the CLI) ----
PART  = "all";   // "all" | "stage2up" | "stage3up" | "head"
SCALE = 1;       // 1 = metres (model units); 1000 = millimetres

eps = 0.003;     // small overlap so stacked sections fuse into one manifold solid

// ---- Stage cylinders (computed in main.py: motor-case dia + stage length) ----
d1 = 1.57; L1 = 7.70;   // Stage 1
d2 = 1.16; L2 = 4.45;   // Stage 2
d3 = 0.83; L3 = 2.83;   // Stage 3

// ---- Interstage shroud = skirt (lower dia) + cone; belongs to the LOWER stage,
//      encloses the upper stage's nozzles, drops with the lower stage. ----
H_sk12 = 0.40; H_cone21 = 0.40;   // stage 1/2 interstage (d1 -> d2)
H_sk23 = 0.30; H_cone32 = 0.35;   // stage 2/3 interstage (d2 -> d3)

// ---- Navigation + head (read from the CAD drawing; edit freely) ----
d_ns = 0.70; L_ns = 0.75;   // navigation module
d_pl = 0.54; L_pl = 0.50;   // head / warhead body
h_nose = 0.51;              // warhead nose cone
H_s3nav   = 0.18;           // adapter stage 3 -> nav  (0.83 -> 0.70)
H_navhead = 0.18;           // adapter nav -> head     (0.70 -> 0.54)

// ---- Nozzles: 4 per stage, after-critical (divergent) cone only, from main.py ----
//      each bell = throat d_kr (top) -> exit d_a (bottom), length l_a   (3.31-3.39)
n_noz      = 4;
noz_gap    = 1.06;                   // >1 keeps the 4 bells distinct (genus-0 mesh)
noz_throat = [0.167, 0.135, 0.086];  // d_kr  throat dia   (stage 1/2/3)
noz_exit   = [0.450, 0.566, 0.444];  // d_a   exit dia
noz_bell   = [0.389, 0.593, 0.492];  // l_a   after-critical (divergent) length

// ---- Colours (preview only; STL has no colour) ----
C_S1 = [0.70, 0.70, 0.72];
C_S2 = [0.76, 0.76, 0.78];
C_S3 = [0.82, 0.82, 0.84];
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
    de = noz_exit[st]; dt = noz_throat[st]; h = noz_bell[st];
    br = de / sqrt(2) * noz_gap;   // bolt circle: 4 bells just clear of each other
    for (i = [0 : n_noz - 1])
        rotate([0, 0, 45 + i * 360 / n_noz])
            translate([br, 0, z_top - h])
                cylinder(d1 = de, d2 = dt, h = h + eps);
}

// ---------- cumulative z (stage-1 base at z = 0) ----------
z_body1  = 0;
z_sk12   = z_body1 + L1;       // interstage skirt 1-2 (d1, encloses stage-2 nozzles)
z_cone21 = z_sk12 + H_sk12;    // cone d1 -> d2
z_body2  = z_cone21 + H_cone21;
z_sk23   = z_body2 + L2;       // interstage skirt 2-3 (d2, encloses stage-3 nozzles)
z_cone32 = z_sk23 + H_sk23;    // cone d2 -> d3
z_body3  = z_cone32 + H_cone32;
z_s3nav  = z_body3 + L3;
z_nav    = z_s3nav + H_s3nav;
z_navhead = z_nav + L_ns;
z_head   = z_navhead + H_navhead;
z_nose   = z_head + L_pl;
z_top    = z_nose + h_nose;

// ---------- section groups (defined in global coordinates) ----------
// Each group carries its own (bare) nozzles + the interstage shroud ABOVE it that
// encloses the NEXT stage's nozzles. Dropping a group exposes the next stage's nozzles.

module stage1_group() {
    color(C_STR) nozzles(z_body1, 0);              // bare (no skirt on stage 1)
    color(C_S1)  seg_tube(z_body1, L1, d1);
    color(C_STR) {
        seg_tube(z_sk12, H_sk12, d1);              // skirt enclosing stage-2 nozzles
        seg_cone(z_cone21, H_cone21, d1, d2);
    }
}

module stage2_group() {
    color(C_STR) nozzles(z_body2, 1);              // exposed once stage 1 drops
    color(C_S2)  seg_tube(z_body2, L2, d2);
    color(C_STR) {
        seg_tube(z_sk23, H_sk23, d2);              // skirt enclosing stage-3 nozzles
        seg_cone(z_cone32, H_cone32, d2, d3);
    }
}

module stage3_group() {
    color(C_STR) nozzles(z_body3, 2);              // exposed once stage 2 drops
    color(C_S3)  seg_tube(z_body3, L3, d3);
}

module nav_group() {
    color(C_STR) seg_cone(z_s3nav, H_s3nav, d3, d_ns);
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

// aft-most z of each configuration (nozzle exit plane), to re-zero export to z = 0
function part_base_z(part) =
    part == "head"     ? z_head :
    part == "stage3up" ? z_body3 - noz_bell[2] :
    part == "stage2up" ? z_body2 - noz_bell[1] :
                         z_body1 - noz_bell[0];

scale(SCALE) translate([0, 0, -part_base_z(PART)]) assembly(PART);

// ---------- verification echo (Console window) ----------
echo(str("PART=", PART, "  SCALE=", SCALE));
echo(str("Stage 1: d=", d1, " L=", L1));
echo(str("Stage 2: d=", d2, " L=", L2));
echo(str("Stage 3: d=", d3, " L=", L3));
echo(str("Nav    : d=", d_ns, " L=", L_ns));
echo(str("Head   : d=", d_pl, " L=", L_pl, " + nose ", h_nose));
echo(str("Nozzles/stage=", n_noz, "  throat=", noz_throat, " exit=", noz_exit, " bell=", noz_bell));
echo(str("Full height (excl. nozzles) = ", z_top, " m"));
