import json
import math
import sys
from pathlib import Path
from typing import NamedTuple

import numpy as np

from typst import (
    DASH,
    emit,
    fmt,
    param_row,
    param_rows,
    param_table,
    section,
    stage_header,
)
from utils import (
    G0,
    burn_rate,
    combustion_temp,
    fuel_props,
    interp_chart,
    k0_from_k,
    l_coefficient,
    load_materials,
    load_trajectory,
    specific_thrust_corrected,
    specific_thrust_design,
    specific_thrust_ground,
    specific_thrust_vacuum,
)

# Primary design requirement: full flight range L (km).
L_FULL = 12053

# Velocity-loss coefficient on the active trajectory segment (k_V): the
# characteristic (Tsiolkovsky) velocity exceeds the required burnout velocity
# V_к by this factor to cover gravity/drag/steering losses.
#
# Аппазов §2.5 (2.128) recommends k_V = 1.15…1.25 for L = 10…14 тыс. км, with
# larger ranges taking the SMALLER values, and his §5.2 РДТТ example adopts
# 1.165 at L = 10000 км. By that rule L = 12053 км argues for ≤ 1.165.
#
# The §4.4 проверочный расчет disagrees. Sweeping the design k_V and measuring
# the loss factor the simulator actually achieves gives measured > design
# everywhere inside the band (1.165→1.230, 1.211→1.244, 1.241→1.251,
# 1.248→1.260), so no self-consistent value exists within it: sizing for the
# band's own losses always under-sizes the vehicle.
#
# With the 8-arc pitch program, 1.25 reaches L_FULL: 12053 km with all four
# §4.4 limits satisfied, at the chart λ_з. 1.25 is the TOP of the §2.5 band but
# inside it. The 4-arc program needed 1.275 — outside the band — so the extra
# pitch arcs are what keep k_V admissible at all. Still contrary to the §2.5
# monotonicity hint that L = 12053 км wants the lower end, so the РПЗ should
# explain why the achieved losses land high: the α-limited gravity turn and a
# CFD table still tied to the older, wider geometry are the honest reasons.
K_V = 1.25

# Burnout-trajectory reference (table 2.1, assets/table-2.1.csv): full
# range L (km) maps to burnout altitude h_к (km), active-segment range l_к (km),
# path angle θ_к (deg), burnout velocity V_к (m/s) and range gradient L'_V.
_TRAJ = load_trajectory()

# Per-stage input data. d_m — motor outer diameter (m), taken from prototype.
# mu_k — burnout mass fraction μ_к (fuel burned / stage launch mass).
# l_z — charge elongation λ_з; optional. Chart 4-27 gives a recommended value
# from ρ_т·u, which is what a stage uses when the key is absent. Overriding it
# shortens and fattens the motor: d_м ∝ λ_з^(-1/3), so the burn time (3.22)
# lengthens and the thrust-to-weight ratio n_0 = P_уд.0 μ_к / Δt_к drops.
# No stage overrides it: every stage uses the chart value. A sweep of stage 1
# over λ_з ∈ 3.5…8.0 with the real CFD table did show a ~4 % better plateau
# around 5…7 (the drag-free sweep had shown the opposite), but with the 8-arc
# pitch program the chart value 4.16 reaches L_FULL on its own, so the
# deviation is no longer needed and the chart recommendation stands.
STAGES = [
    {"p_k": 50, "p_a": 0.70, "fuel": "polybutadiene", "d_m": 1.7, "mu_k": 0.66},
    {"p_k": 35, "p_a": 0.37, "fuel": "polyurethane", "d_m": 1.1, "mu_k": 0.65},
    {"p_k": 35, "p_a": 0.14, "fuel": "polyurethane", "d_m": 0.9, "mu_k": 0.65},
]

# Payload carried by the top stage: warhead m_бч plus control unit m_ау (kg).
M_BCH = 500
M_AU = 120

# Trajectory-simulator config kept in sync by sync_traj_config().
TRAJ_CONFIG_PATH = "traj/rocket.json"

# -------------------------------------------------------------------
# Material properties (assets/materials.csv)
# -------------------------------------------------------------------
_MAT = load_materials()
SIGMA_V = _MAT["case_sigma"] * 1e6  # ultimate tensile strength σ_в, Pa
RHO_M = _MAT["case_rho"]  # case material density ρ_м, kg/m³
RHO_BR = _MAT["armor_rho"]  # armor/adhesive density ρ_бр, kg/m³
RHO_TZ = _MAT["tz_rho"]  # heat-protection density ρ_тз, kg/m³
RHO_C_AVG = _MAT["nozzle_rho"]  # average nozzle density ρ_c^ср, kg/m³
A_TZ = f"a_tz={_MAT['tz_a'] * 1e6:g}e-6"  # heat-protection curve label (chart-3-6)

# -------------------------------------------------------------------
# Weight-coefficient design factors (table 3.4 / prototype data)
# -------------------------------------------------------------------
ETA = 1.2  # safety factor η (1.2–1.5)
ALPHA_BR = 0.07  # armor fraction α_бр
EPSILON = 0.99  # outer-surface coating factor ε
D_K_BAR = 0.3  # normalized inner bore d̄_к
ALPHA_C = 0.005  # nozzle mass coefficient α_c
BETA_C_DEG = 20  # nozzle half-angle β_c, degrees
BETA_C = math.radians(BETA_C_DEG)
A_OMEGA_3 = 0.025  # guarantee fuel reserve coefficient α_ω (3rd stage only)
N_TAIL = 0.012  # tail-section mass coefficient N

# Charge/nozzle geometry factors (section 3.28–3.42)
K_S = 2.03  # burning-surface shape coefficient k_s (2.03–3.4)
N_NOZZLES = 4  # number of nozzles per stage n_с (prototype: 4 gimbaled nozzles)
H_RUDDER = 0.2  # end-rudder protrusion h, m

CHART_FA = "assets/chart-3-5-fa-fkp-pa-pk.csv"
CHART_DTZ = "assets/chart-3-6-delta-tz-d-m.csv"


class StageProps(NamedTuple):
    """Fuel and chart lookups for one stage, resolved once and shared by the
    thrust, weight and geometry sections so they cannot drift apart."""

    P_ud_st: float  # standard specific thrust P_уд.ст, s
    al_pct: int  # aluminum content, %
    rho_t: float  # propellant density ρ_т, kg/m³
    R: float  # gas constant R, J/(kg·K)
    k: float  # ratio of specific heats k
    T_st: float  # standard combustion temperature, K
    T: float  # actual combustion temperature at p_k, K
    u: float  # linear burn rate u, mm/s
    rho_u: float  # mass burn rate ρ_т·u, kg/(m²·s)
    l_z: float  # charge elongation λ_з (chart 4-27)
    K0: float  # nozzle flow coefficient K_0 (table 3.10)
    fa_fkp: float  # nozzle area ratio F_a/F_кр (chart 3-5)
    delta_tz_mm: float  # heat-protection thickness δ_тз at the prototype d_м, mm


class Thrust(NamedTuple):
    """Per-stage specific-thrust results (seconds, except T in kelvin)."""

    P_ud_pr: float  # corrected standard specific thrust
    P_ud_r: float  # design-condition specific thrust
    T: float  # combustion temperature, K
    P_ud_0: float  # ground-level specific thrust P_уд.0
    P_ud_v: float  # vacuum specific thrust


class Weight(NamedTuple):
    """Per-stage weight-coefficient results (kg/m³, except the dimensionless
    l_z and a_dv)."""

    a: float  # case and bottoms
    b: float  # armor coating and adhesive
    c: float  # nozzle with heat protection
    q: float  # heat-protection coefficient
    psi: float  # propellant charge coefficient
    l_z: float  # charge elongation λ_з
    a_dv: float  # motor structural coefficient


class Subrockets(NamedTuple):
    """Per-stage subrocket sizing results (3.19–3.25)."""

    m0: list[float]  # subrocket launch masses m_(0 i), kg
    omega_z: list[float]  # propellant charge masses ω_(з i), kg
    dt: list[float]  # motor burn times Δt_(к i), s
    d_m: list[float]  # motor diameters d_(м i), m


class Geometry(NamedTuple):
    """Per-stage charge and nozzle geometry results (3.28–3.42), metres unless
    noted."""

    l_zi: float  # charge length l_з
    h_slot: float  # slot height h
    S: float  # burning surface S, m²
    d_kr2: float  # throat diameter squared, m²
    d_kr: float  # throat diameter d_кр
    F_kr: float  # throat area, m²
    F_a: float  # exit area, m²
    d_a: float  # exit diameter d_a
    l_v: float  # igniter length l_в
    delta_k: float  # case wall thickness δ_к
    d_z: float  # charge diameter d_з
    d_k: float  # channel diameter d_к
    l_dk: float  # nozzle convergent length l_дк
    l_a: float  # nozzle divergent length l_a
    d_v: float  # igniter diameter d_в
    l_dn: float  # bottoms length l_дн
    L: float  # overall stage length L


def stage_props(s: dict, i: int) -> StageProps:
    """Resolve every fuel/chart lookup a stage needs.

    δ_тз is read at the *prototype* motor diameter s["d_m"], not the computed
    d_(м i), so the heat-protection thickness stays consistent between the
    weight coefficients (3.10) and the geometry section (3.37).
    """
    props = fuel_props(s["fuel"])
    p_k = s["p_k"]
    k = float(props["k_st"])
    T_st = float(props["T"])
    u, rho_u = burn_rate(s["fuel"], p_k)  # u [mm/s], rho_u [kg/(m²·s)]
    return StageProps(
        P_ud_st=float(props["P_ud"]),
        al_pct=int(props["al_pct"]),
        rho_t=float(props["rho"]),
        R=float(props["R_st"]),
        k=k,
        T_st=T_st,
        T=combustion_temp(T_st, p_k),
        u=u,
        rho_u=rho_u,
        l_z=s.get("l_z", l_coefficient(rho_u, i)),
        K0=k0_from_k(k),
        fa_fkp=interp_chart(CHART_FA, f"k{k:.2f}", p_k / s["p_a"]),
        delta_tz_mm=interp_chart(CHART_DTZ, A_TZ, s["d_m"]),
    )


PROPS = [stage_props(s, i) for i, s in enumerate(STAGES, 1)]


def traj_ref(col: str) -> float:
    """Table 2.1 reference value at the design range L_FULL."""
    return float(np.interp(L_FULL, _TRAJ["L"], _TRAJ[col]))


def calc_thrust(i: int) -> Thrust:
    s, p = STAGES[i - 1], PROPS[i - 1]
    p_k, p_a = s["p_k"], s["p_a"]

    P_ud_pr = specific_thrust_corrected(p.P_ud_st, p.al_pct)
    P_ud_r = specific_thrust_design(P_ud_pr, p_k, p_a)
    P_ud_0 = specific_thrust_ground(P_ud_pr, p_k)
    P_ud_v = specific_thrust_vacuum(P_ud_r, p.R, p.T, p.k, p_a, p_k)

    # Equation 1: corrected standard specific thrust
    if p.al_pct == 0:
        emit(f'P^"пр"_"уд {i}" = 0.96 dot {fmt(p.P_ud_st)} = {P_ud_pr:.2f} "с"')
    else:
        emit(
            f'P^"пр"_"уд {i}" = {fmt(p.P_ud_st)} dot '
            f"[1-(4.3+0.17dot {p.al_pct}+0.009dot {p.al_pct}^2) dot 10^(-2)]"
            f' = {P_ud_pr:.2f} "с"'
        )

    # Equation 2: design-condition specific thrust
    emit(
        f'P^"р"_"уд {i}" = {P_ud_pr:.2f} + 19.4 + 0.76 dot {fmt(p_k)}'
        f" - 0.003 dot {fmt(p_k)}^2 - 70 dot {p_a} + 25 dot {p_a}^2"
        f' = {P_ud_r:.2f} "с"'
    )

    # Equation 3: combustion temperature
    emit(f'T_{i} = {fmt(p.T_st)} + 1.12 dot ({fmt(p_k)} - 40) = {p.T:.1f} "К"')

    # Ground-level specific thrust: formula (3.3) at p_a = 1 bar. Only the first
    # stage needs it — it is the P_уд.01 term of (3.6) — so only that one is
    # emitted, matching the dashes in the summary table.
    if i == 1:
        emit(
            f'P_"уд.0{i}" = {P_ud_pr:.2f} + 19.4 + 0.76 dot {fmt(p_k)}'
            f" - 0.003 dot {fmt(p_k)}^2 - 70 dot 1 + 25 dot 1^2"
            f' = {P_ud_0:.2f} "с"'
        )

    # Equation 4: vacuum specific thrust
    emit(
        f'P^"п"_"уд {i}" = {P_ud_r:.2f}'
        f" + ({fmt(p.R)} dot {p.T:.1f})/({G0}^2 dot {P_ud_r:.2f})"
        f" dot ({p_a}/{fmt(p_k)})^(({p.k}-1)/{p.k})"
        f' = {P_ud_v:.2f} "с"'
    )

    print()
    return Thrust(P_ud_pr, P_ud_r, p.T, P_ud_0, P_ud_v)


def calc_weights(i: int) -> Weight:
    s, p = STAGES[i - 1], PROPS[i - 1]
    p_k = s["p_k"]
    l_z = p.l_z

    # delta_tz from chart-3-6 (mm), converted to dimensionless K_тз = δ_тз / d_м
    K_tz = (p.delta_tz_mm * 1e-3) / s["d_m"]

    # (3.7) case and bottoms; η (safety factor) multiplies the mass
    a = (math.pi / 2 * l_z + 1) * (p_k * 1e5 * RHO_M) / SIGMA_V * ETA
    emit(
        f"a_{i} = (pi/2 dot {l_z:.1f} + 1)"
        f" dot ({fmt(p_k)} dot 10^5 dot {fmt(RHO_M)})"
        f" / ({SIGMA_V / 1e6:.0f} dot 10^6) dot {ETA}"
        f' = {a:.1f} "кг/м³"'
    )

    # (3.8) armor coating and adhesive; u in mm/s per formula notation
    b = (
        math.pi
        / 2
        * RHO_BR
        * (ALPHA_BR / (2 * p.u) * (1 - D_K_BAR**2) + l_z * (1 - EPSILON))
    )
    emit(
        f"b_{i} = pi/2 dot {fmt(RHO_BR)}"
        f" dot [{ALPHA_BR} / (2 dot {p.u:.2f}) dot (1 - {D_K_BAR}^2)"
        f" + {l_z:.1f} dot (1 - {EPSILON})]"
        f' = {b:.1f} "кг/м³"'
    )

    # (3.9) nozzle with heat protection. The leading 2.03 is the nozzle-mass
    # constant of this formula — it merely coincides numerically with K_S, the
    # burning-surface coefficient of (3.30), and must not be folded into it.
    c = (
        2.03
        * p.rho_u
        * RHO_C_AVG
        * math.sqrt(p.R * p.T)
        / (p.K0 * p_k * 1e5 * math.sin(BETA_C))
        * (p.fa_fkp - 1)
        * l_z
        * ALPHA_C
    )
    emit(f'F_("а{i}") / F_"кр" = {p.fa_fkp:.2f}')
    emit(
        f"c_{i}"
        f" = (2.03 dot {p.rho_u:.2f} dot {fmt(RHO_C_AVG)} dot sqrt({fmt(p.R)} dot {p.T:.1f}))"
        f" / ({p.K0:.3f} dot {fmt(p_k)} dot 10^5 dot sin({BETA_C_DEG}°))"
        f" dot ({p.fa_fkp:.2f} - 1) dot {l_z:.1f} dot {ALPHA_C}"
        f' = {c:.1f} "кг/м³"'
    )

    # (3.10) heat-protection thickness ratio
    emit(f'K_("тз"{i}) = {p.delta_tz_mm:.2f} dot 10^(-3) / {s["d_m"]} = {K_tz:.5f}')

    # (3.11) heat-protection coefficient
    q = K_tz * (1.96 + math.pi * (0.37 * l_z - 0.30)) * RHO_TZ
    emit(
        f"q_{i} = {K_tz:.5f}"
        f" dot [1.96 + pi (0.37 dot {l_z:.1f} - 0.30)]"
        f" dot {fmt(RHO_TZ)}"
        f' = {q:.1f} "кг/м³"'
    )

    # (3.12) propellant charge coefficient ψ
    psi = math.pi / 4 * (1 - D_K_BAR**2) * p.rho_t
    emit(f'psi_{i} = pi/4 (1 - {D_K_BAR}^2) dot {fmt(p.rho_t)} = {psi:.1f} "кг/м³"')

    # (3.13) motor structural coefficient
    a_dv = (a + b + c + q) / (psi * l_z)
    emit(
        f'a_("дв{i}") = ({a:.1f} + {b:.1f} + {c:.1f} + {q:.1f})'
        f" / ({psi:.1f} dot {l_z:.1f})"
        f" = {a_dv:.4f}"
    )

    print()
    return Weight(a, b, c, q, psi, l_z, a_dv)


def calc_geometry(i: int, d: float) -> Geometry:
    """Charge and nozzle geometry for one stage at its computed diameter d."""
    s, p = STAGES[i - 1], PROPS[i - 1]
    p_k = s["p_k"]
    l_z = p.l_z
    cot_beta = 1 / math.tan(BETA_C)

    l_zi = l_z * d  # (3.28) charge length
    h_slot = (0.37 * l_z - 0.30) * d  # (3.29) slot height
    S = K_S * l_z * d**2  # (3.30) burning surface
    d_kr2 = (4 * S * p.rho_u * math.sqrt(p.R * p.T)) / (
        math.pi * p.K0 * p_k * 1e5 * N_NOZZLES
    )  # (3.31) throat diameter squared
    d_kr = math.sqrt(d_kr2)
    F_kr = math.pi * d_kr2 / 4  # (3.32) throat area
    F_a = p.fa_fkp * F_kr  # (3.33) exit area
    d_a = math.sqrt(4 * F_a / math.pi)  # (3.34) exit diameter
    l_v = 0.1 * d  # (3.35) igniter length
    # Case thickness from thin-walled hoop-stress condition (matches a, 3.7)
    delta_k = ETA * p_k * 1e5 * d / (2 * SIGMA_V)
    d_z = d - 2 * delta_k - 2 * p.delta_tz_mm * 1e-3  # (3.37) charge diameter
    d_k = D_K_BAR * d_z  # (3.36) channel diameter
    l_dk = (d_k - d_kr) / 2 * cot_beta  # (3.38) convergent length
    l_a = (d_a - d_kr) / 2 * cot_beta  # (3.39) divergent length
    d_v = 0.2 * d  # (3.40) igniter diameter
    l_dn = 0.3 * d  # (3.41) bottoms length
    L = l_zi + l_a + l_dk + H_RUDDER + l_v  # (3.42) stage length

    return Geometry(
        l_zi=l_zi, h_slot=h_slot, S=S, d_kr2=d_kr2, d_kr=d_kr, F_kr=F_kr,
        F_a=F_a, d_a=d_a, l_v=l_v, delta_k=delta_k, d_z=d_z, d_k=d_k,
        l_dk=l_dk, l_a=l_a, d_v=d_v, l_dn=l_dn, L=L,
    )  # fmt: skip


def emit_thrust() -> tuple[list[Thrust], float]:
    """Specific-thrust section: per-stage equations, summary table, P_уд.ср."""
    section("Удельная тяга", "(3.1) - (3.6)")
    thrust = []
    for i in range(1, len(STAGES) + 1):
        stage_header(i)
        thrust.append(calc_thrust(i))

    entries = [
        ('$P_"уд.ст"^"пр"$, с', ".2f", "P_ud_pr"),
        ('$P_"уд"^"р"$, с', ".2f", "P_ud_r"),
        ("$T$, К", ".1f", "T"),
    ]
    rows = param_rows(entries, thrust)
    # P_уд.0 is only used for the first stage (the P_уд.01 term of 3.6); the
    # upper stages never fire at sea level, so their cells are dashed.
    rows.append(param_row('$P_"уд.0"$, с', [f"${thrust[0].P_ud_0:.2f}$", DASH, DASH]))
    rows.append(param_row('$P_"уд.п"$, с', [t.P_ud_v for t in thrust], ".2f"))
    param_table(rows)
    print()

    P_ud_01 = thrust[0].P_ud_0
    P_ud_v1, P_ud_v2, P_ud_v3 = (t.P_ud_v for t in thrust)
    P_ud_avg = (((P_ud_01 + P_ud_v1) / 2) + P_ud_v2 + P_ud_v3) / 3
    emit(
        f'P_"уд.ср" = 1/3 (({P_ud_01:.2f}+{P_ud_v1:.2f})/2'
        f"+{P_ud_v2:.2f}+{P_ud_v3:.2f})"
        f' = {P_ud_avg:.2f} "с"'
    )
    print()
    return thrust, P_ud_avg


def emit_weights() -> tuple[list[Weight], list[float]]:
    """Weight-coefficient section: per-stage equations and the K_i table."""
    section("Весовые коэффициенты", "(3.7) - (3.14)")
    weight = []
    for i in range(1, len(STAGES) + 1):
        stage_header(i)
        weight.append(calc_weights(i))

    entries = [
        ("$a_i$, кг/м³", ".1f", "a"),
        ("$b_i$, кг/м³", ".1f", "b"),
        ("$c_i$, кг/м³", ".1f", "c"),
        ("$q_i$, кг/м³", ".1f", "q"),
        ("$psi_i$, кг/м³", ".1f", "psi"),
        ('$a_("дв"i)$', ".4f", "a_dv"),
    ]
    rows = param_rows(entries, weight)
    # α_ω — only the 3rd stage carries a guarantee fuel reserve
    a_omega_stages = [0.0, 0.0, A_OMEGA_3]
    a_omega_cells = [f"${v:.4f}$" if v else DASH for v in a_omega_stages]
    rows.append(param_row("$a_(omega i)$", a_omega_cells))
    # N — same tail-section coefficient for every stage
    rows.append(param_row("$N_i$", [f"${N_TAIL:.4f}$" for _ in STAGES]))
    # K — engine coefficient: a_дв plus the guarantee fuel reserve
    k_values = [w.a_dv + ao for w, ao in zip(weight, a_omega_stages)]
    rows.append(param_row("$K_i$", [f"${v:.4f}$" for v in k_values]))
    param_table(rows)
    print()

    emit(f"a_(omega 3) = {A_OMEGA_3}")
    emit(f"N_i = {N_TAIL}")
    print()
    return weight, k_values


def emit_trajectory(thrust: list[Thrust], P_ud_avg: float) -> float:
    """Active-segment section: end-of-burn parameters, V_к and relative weights."""
    section("Параметры в конце активного участка", "(3.15) - (3.18)")
    emit(
        f'h_"к" approx {fmt(traj_ref("h_k"))} "км", quad '
        f'l_"к" approx {fmt(traj_ref("l_k"))} "км", quad '
        f'theta.alt_"к" approx {fmt(traj_ref("theta_k"))} degree'
    )
    print()

    # (3.16) Achievable burnout velocity from the prototype μ_кi: the
    # characteristic (Tsiolkovsky) velocity Σ g₀ P_уд.п ln(1/(1-μ_к)) divided by
    # the loss factor k_V. The prototype μ_кi are only a starting guess.
    mu_proto = [s["mu_k"] for s in STAGES]
    p_ud_p = [t.P_ud_v for t in thrust]
    n = len(STAGES)
    char_v = sum(G0 * p_ud_p[i] * math.log(1 / (1 - mu_proto[i])) for i in range(n))
    v_k_ach = char_v / K_V
    sum_body = " + ".join(
        f"{G0} dot {p_ud_p[i]:.2f} dot ln 1/(1-{fmt(mu_proto[i])})" for i in range(n)
    )
    emit(f'V_"к" = 1/{K_V} ({sum_body}) = {v_k_ach:.0f} "м/с"')

    # (3.17) Velocity the design range actually demands, read from table 3.12.
    # (3.18) below is driven by THIS value, not by v_k_ach: μ_кi is being chosen
    # to meet the requirement, exactly as in the Аппазов §5.2 worked example
    # (1.165 · 6940 = 8100 м/с → μ_кi = 0.647 at L = 10000 км). Feeding the
    # achievable velocity back in instead would cancel k_V algebraically and
    # leave both k_V and L_FULL with no effect on any dimension.
    v_k_req = traj_ref("V_k")
    emit(f'V_"к.потр" = {v_k_req:.0f} "м/с"')
    short = v_k_req - v_k_ach
    emit(
        f'Delta V = V_"к.потр" - V_"к" = {v_k_req:.0f} - {v_k_ach:.0f}'
        f' = {short:+.0f} "м/с"'
    )

    # Velocity demand including losses: V_к + ΔV_к = k_V·V_к.потр.
    emit(
        f'V_"к" + Delta V_"к" = k_V V_"к.потр"'
        f' = {K_V} dot {v_k_req:.0f} = {K_V * v_k_req:.0f} "м/с"'
    )
    print()

    # (3.18) Relative fuel weights of the subrockets: the required characteristic
    # velocity k_V·V_к.потр is split equally across the n stages.
    mu_calc = 1 - math.exp(-(K_V * v_k_req) / (n * G0 * P_ud_avg))
    emit(
        f'mu_("к"i) = 1 - exp(-({K_V} dot {v_k_req:.0f})'
        f"/({n} dot {G0} dot {P_ud_avg:.2f})) = {mu_calc:.3f}"
    )
    print()
    return mu_calc


def emit_masses(
    weight: list[Weight], k_values: list[float], mu_k: float, thrust: list[Thrust]
) -> Subrockets:
    """Subrocket masses (3.19/3.20), motor diameters (3.21), burn times (3.22),
    thrust-to-weight (3.23), midsection load (3.24), propellant mass (3.25),
    thrust (3.26), and stage lengths (3.27)."""
    section("Массы и геометрия субракет", "(3.19) - (3.27)")
    n = len(STAGES)

    # ---- Compute all values (m0 top-down, rest in stage order) ----
    m0 = [0.0] * n
    payload = [0.0] * n
    for i in range(n - 1, -1, -1):
        payload[i] = (M_BCH + M_AU) if i == n - 1 else m0[i + 1]
        m0[i] = payload[i] / (1 - N_TAIL - (1 + k_values[i]) * mu_k)

    d_m = []
    for i in range(n):
        w = weight[i]
        d = (((1 - N_TAIL) * m0[i] - payload[i]) / ((1 + w.a_dv) * w.psi * w.l_z)) ** (
            1 / 3
        )
        d_m.append(d)

    dt = [d_m[i] * (1 - D_K_BAR) / (2 * PROPS[i].u * 1e-3) for i in range(n)]
    # Stage 1 is rated at ground level (subscript 0 in 3.23/3.26), the upper
    # stages in vacuum (subscript п) — p_ud must follow lam_sub.
    p_ud = [thrust[0].P_ud_0] + [thrust[i].P_ud_v for i in range(1, n)]
    lam_sub = ["0"] + ['"п"'] * (n - 1)
    lam = [dt[i] / (mu_k * p_ud[i]) for i in range(n)]
    p_m1 = 4 * m0[0] / (math.pi * d_m[0] ** 2)
    omega_z = [mu_k * m0[i] for i in range(n)]
    m_dot = [omega_z[i] / dt[i] for i in range(n)]
    P_r1 = thrust[0].P_ud_0 * G0 * m_dot[0]
    P_v = [thrust[i].P_ud_v * G0 * m_dot[i] for i in range(n)]
    l_k = [1.15 * weight[i].l_z * d_m[i] for i in range(n)]

    # ---- Print per stage ----
    for i in range(n):
        stage_header(i + 1)
        denom_src = f"1 - {N_TAIL} - (1 + {k_values[i]:.4f}) dot {mu_k:.3f}"
        num_src = f"({M_BCH} + {M_AU})" if i == n - 1 else f"{m0[i + 1]:.0f}"
        emit(f'm_(0 {i + 1}) = {num_src}/({denom_src}) = {m0[i]:.0f} "кг"')
        w = weight[i]
        emit(
            f"d_(м {i + 1}) = root(3, ((1 - {N_TAIL}) dot {m0[i]:.0f} - {payload[i]:.0f})"
            f"/((1 + {w.a_dv:.4f}) dot {w.psi:.1f} dot {w.l_z:.1f}))"
            f' = {d_m[i]:.2f} "м"'
        )
        emit(
            f"Delta t_(к {i + 1}) = ({d_m[i]:.2f} dot (1 - {D_K_BAR}))"
            f'/(2 dot {PROPS[i].u:.2f} dot 10^(-3)) = {dt[i]:.1f} "с"'
        )
        emit(
            f"lambda_({lam_sub[i]}{i + 1}) = {dt[i]:.1f}"
            f"/({mu_k:.3f} dot {p_ud[i]:.2f}) = {lam[i]:.3f}"
        )
        if i == 0:
            emit(
                f'P_("м"1) = (4 dot {m0[0]:.0f})/(pi dot {d_m[0]:.2f}^2)'
                f' = {p_m1:.0f} "кг/м²"'
            )
        emit(f'omega_(з {i + 1}) = {mu_k:.3f} dot {m0[i]:.0f} = {omega_z[i]:.0f} "кг"')
        if i == 0:
            emit(
                f"P_(0 1) = {thrust[0].P_ud_0:.2f} dot {G0} dot {omega_z[0]:.0f} / {dt[0]:.1f}"
                f' = {P_r1 / 1000:.1f} "кН"'
            )
        emit(
            f'P_("п"{i + 1}) = {thrust[i].P_ud_v:.2f} dot {G0} dot {omega_z[i]:.0f} / {dt[i]:.1f}'
            f' = {P_v[i] / 1000:.1f} "кН"'
        )
        emit(
            f"l_(к {i + 1}) approx 1.15 dot {weight[i].l_z:.1f} dot {d_m[i]:.2f}"
            f' = {l_k[i]:.2f} "м"'
        )
        print()

    rows = [
        param_row("$m_(0 i)$, кг", m0, ".0f"),
        param_row("$d_(м i)$, м", d_m, ".2f"),
        param_row("$Delta t_(к i)$, с", dt, ".1f"),
        param_row("$lambda_([0\\/п] i)$", lam, ".3f"),
        param_row("$omega_(з i)$, кг", omega_z, ".0f"),
        param_row("$P_(0 1)$, кН", [f"${P_r1 / 1000:.1f}$", DASH, DASH]),
        param_row('$P_("п" i)$, кН', [v / 1000 for v in P_v], ".1f"),
        param_row("$l_(к i)$, м", l_k, ".2f"),
    ]
    param_table(rows)
    print()
    return Subrockets(m0=m0, omega_z=omega_z, dt=dt, d_m=d_m)


def emit_geometry(d_m: list[float]) -> None:
    """Charge and nozzle geometry (3.28–3.42): charge length, slot height and
    burning surface (3.28–3.30); throat/exit areas and diameters (3.31–3.34);
    igniter, charge and channel diameters (3.35–3.37); nozzle convergent and
    divergent lengths, igniter/bottoms sizes and overall stage lengths
    (3.38–3.42)."""
    section("Геометрия зарядов и сопел", "(3.28) - (3.43)")
    geo = [calc_geometry(i, d_m[i - 1]) for i in range(1, len(STAGES) + 1)]

    # ---- Per-stage numeric results ----
    for i, g in enumerate(geo, 1):
        s, p = STAGES[i - 1], PROPS[i - 1]
        d = d_m[i - 1]
        stage_header(i)
        emit(f'l_(з {i}) = {p.l_z:.1f} dot {d:.2f} = {g.l_zi:.2f} "м"')
        emit(
            f"h_{i} = (0.37 dot {p.l_z:.1f} - 0.30) dot {d:.2f}"
            f' = {g.h_slot:.3f} "м"'
        )
        emit(f'S_{i} = {K_S} dot {p.l_z:.1f} dot {d:.2f}^2 = {g.S:.2f} "м²"')
        emit(
            f'd_("кр" {i})^2 = (4 dot {g.S:.2f} dot {p.rho_u:.2f} dot '
            f"sqrt({fmt(p.R)} dot {p.T:.1f}))"
            f"/(pi dot {p.K0:.3f} dot {s['p_k']} dot 10^5 dot {N_NOZZLES})"
            f' = {g.d_kr2:.4f} "м²"'
        )
        emit(f'd_("кр" {i}) = sqrt({g.d_kr2:.4f}) = {g.d_kr:.3f} "м"')
        emit(f'F_("кр" {i}) = (pi dot {g.d_kr2:.4f})/4 = {g.F_kr:.4f} "м²"')
        emit(f'F_(a {i}) = {p.fa_fkp:.2f} dot {g.F_kr:.4f} = {g.F_a:.4f} "м²"')
        emit(f'd_(a {i}) = sqrt((4 dot {g.F_a:.4f})/pi) = {g.d_a:.3f} "м"')
        emit(f'l_(в {i}) = 0.1 dot {d:.2f} = {g.l_v:.3f} "м"')
        emit(
            f"delta_(к {i}) = ({ETA} dot {s['p_k']} dot 10^5 dot {d:.2f})"
            f"/(2 dot {SIGMA_V / 1e6:.0f} dot 10^6)"
            f' = {g.delta_k * 1e3:.2f} dot 10^(-3) "м"'
        )
        emit(f'delta_("тз" {i}) = {p.delta_tz_mm:.2f} dot 10^(-3) "м"')
        emit(
            f"d_(з {i}) = {d:.2f} - 2 dot {g.delta_k * 1e3:.2f} dot 10^(-3)"
            f" - 2 dot {p.delta_tz_mm:.2f} dot 10^(-3)"
            f' = {g.d_z:.3f} "м"'
        )
        emit(f'd_(к {i}) = {D_K_BAR} dot {g.d_z:.3f} = {g.d_k:.3f} "м"')
        emit(
            f'l_("дк" {i}) = (({g.d_k:.3f} - {g.d_kr:.3f})/2)'
            f' dot ctg({BETA_C_DEG}°) = {g.l_dk:.3f} "м"'
        )
        emit(
            f"l_(a {i}) = (({g.d_a:.3f} - {g.d_kr:.3f})/2)"
            f' dot ctg({BETA_C_DEG}°) = {g.l_a:.3f} "м"'
        )
        emit(f'd_(в {i}) = 0.2 dot {d:.2f} = {g.d_v:.3f} "м"')
        emit(f'l_("дн" {i}) approx 0.3 dot {d:.2f} = {g.l_dn:.3f} "м"')
        emit(
            f"L_{i} = {g.l_zi:.2f} + {g.l_a:.3f} + {g.l_dk:.3f}"
            f" + {H_RUDDER} + {g.l_v:.3f}"
            f' = {g.L:.2f} "м"'
        )
        print()

    # ---- Summary table ----
    entries = [
        ("$l_(з i)$, м", ".2f", "l_zi"),
        ("$h_i$, м", ".3f", "h_slot"),
        ("$S_i$, м²", ".2f", "S"),
        ('$d_("кр" i)$, м', ".3f", "d_kr"),
        ('$F_("кр" i)$, м²', ".4f", "F_kr"),
        ("$F_(a i)$, м²", ".4f", "F_a"),
        ("$d_(a i)$, м", ".3f", "d_a"),
        ("$l_(в i)$, м", ".3f", "l_v"),
        ("$d_(з i)$, м", ".3f", "d_z"),
        ("$d_(к i)$, м", ".3f", "d_k"),
        ('$l_("дк" i)$, м', ".3f", "l_dk"),
        ("$l_(a i)$, м", ".3f", "l_a"),
        ("$d_(в i)$, м", ".3f", "d_v"),
        ('$l_("дн" i)$, м', ".3f", "l_dn"),
        ("$L_i$, м", ".2f", "L"),
    ]
    param_table(param_rows(entries, geo))
    print()

    # ---- Condition (3.43): d_м ≥ d_a (1 + √2) for 4-nozzle layout ----
    factor = 1 + math.sqrt(2)
    for i, g in enumerate(geo, 1):
        rhs = g.d_a * factor
        verdict = "проходит" if d_m[i - 1] >= rhs else "не проходит"
        emit(
            f"d_(м {i}) = {d_m[i - 1]:.2f} >= d_(a {i}) (1+sqrt(2))"
            f' = {rhs:.2f} - "{verdict}"'
        )
    print()


def traj_stage_fields(thrust: list[Thrust], sub: Subrockets) -> list[dict]:
    """Per-stage physical fields of traj/rocket.json, rounded to the precision
    the report tables quote so the document and the simulator agree digit for
    digit.

    isp_sl is P_уд.0, the specific thrust against sea-level back pressure,
    which is what traj/model.go's pressure interpolation expects at its lower
    anchor. Only stage 1 fires low enough for it to matter: the upper stages
    ignite at 50 km and 171 km, where ambient is below 0.001 bar and the
    interpolation returns essentially isp_vac regardless."""
    return [
        {
            "m0": round(sub.m0[i]),
            "m_fuel": round(sub.omega_z[i]),
            "burn_time": round(sub.dt[i], 1),
            "isp_sl": round(thrust[i].P_ud_0, 3),
            "isp_vac": round(thrust[i].P_ud_v, 3),
            "dm": round(sub.d_m[i], 2),
        }
        for i in range(len(STAGES))
    ]


def sync_traj_config(thrust: list[Thrust], sub: Subrockets, write: bool) -> None:
    """Keep the physical fields of traj/rocket.json in sync with this script.

    This script owns payload_mass and the per-stage m0/m_fuel/burn_time/
    isp_sl/isp_vac/dm. Everything else in that file — t_vertical, the pitch
    arcs, limits and the part keys — is optimizer output and is preserved
    verbatim.

    Without ``write`` the file is only checked, and any drift is reported on
    stderr so that piping stdout into the Typst report stays unaffected."""
    path = Path(TRAJ_CONFIG_PATH)
    if not path.exists():
        return

    cfg = json.loads(path.read_text(encoding="utf-8"))
    payload = round(M_BCH + M_AU)
    fields = traj_stage_fields(thrust, sub)

    drift = []
    if cfg.get("payload_mass") != payload:
        drift.append(f"payload_mass: {cfg.get('payload_mass')} -> {payload}")
    for i, (stage, want) in enumerate(zip(cfg.get("stages", []), fields), start=1):
        for key, value in want.items():
            if stage.get(key) != value:
                drift.append(f"stage {i} {key}: {stage.get(key)} -> {value}")

    if not drift:
        return

    if not write:
        print(
            f"warning: {TRAJ_CONFIG_PATH} is out of sync with main.py "
            f"({len(drift)} field(s)); rerun with --write-traj-config",
            file=sys.stderr,
        )
        for line in drift:
            print(f"  {line}", file=sys.stderr)
        return

    cfg["payload_mass"] = payload
    for stage, want in zip(cfg.get("stages", []), fields):
        stage.update(want)
    path.write_text(
        json.dumps(cfg, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(f"updated {TRAJ_CONFIG_PATH} ({len(drift)} field(s))", file=sys.stderr)


def main():
    thrust, P_ud_avg = emit_thrust()
    weight, k_values = emit_weights()
    mu_k = emit_trajectory(thrust, P_ud_avg)
    sub = emit_masses(weight, k_values, mu_k, thrust)
    emit_geometry(sub.d_m)
    sync_traj_config(thrust, sub, write="--write-traj-config" in sys.argv)


if __name__ == "__main__":
    main()
