import json
import math
import sys
from pathlib import Path
from typing import NamedTuple

import numpy as np

from typst import eq, fmt, param_row, param_table, section, stage_header
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
K_V = 1.165

# Burnout-trajectory reference (table 2.1, assets/table-2.1.csv): full
# range L (km) maps to burnout altitude h_к (km), active-segment range l_к (km),
# path angle θ_к (deg), burnout velocity V_к (m/s) and range gradient L'_V.
_TRAJ = load_trajectory()
TRAJ_RANGE = _TRAJ["L"].to_numpy()
TRAJ_HK = _TRAJ["h_k"].to_numpy()
TRAJ_LK = _TRAJ["l_k"].to_numpy()
TRAJ_THETA = _TRAJ["theta_k"].to_numpy()
TRAJ_VK = _TRAJ["V_k"].to_numpy()
TRAJ_LV = _TRAJ["Lv"].to_numpy()

# Per-stage input data. d_m — motor outer diameter (m), taken from prototype.
# mu_k — burnout mass fraction μ_к (fuel burned / stage launch mass).
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
BETA_C = math.radians(20)  # nozzle half-angle β_c
A_OMEGA_3 = 0.025  # guarantee fuel reserve coefficient α_ω (3rd stage only)
N_TAIL = 0.012  # tail-section mass coefficient N

# Charge/nozzle geometry factors (section 3.28–3.42)
K_S = 2.03  # burning-surface shape coefficient k_s (2.03–3.4)
N_NOZZLES = 4  # number of nozzles per stage n_с (prototype: 4 gimbaled nozzles)
H_RUDDER = 0.2  # end-rudder protrusion h, m

CHART_FA = "assets/chart-3-5-fa-fkp-pa-pk.csv"
CHART_DTZ = "assets/chart-3-6-delta-tz-d-m.csv"


class Thrust(NamedTuple):
    """Per-stage specific-thrust results (seconds, except T in kelvin)."""

    P_ud_pr: float  # corrected standard specific thrust
    P_ud_r: float  # design-condition specific thrust
    T: float  # combustion temperature, K
    P_ud_0: float  # ground-level specific thrust P_уд.0
    P_ud_v: float  # vacuum specific thrust


class Weight(NamedTuple):
    """Per-stage weight-coefficient results (kg/m³, except dimensionless a_dv)."""

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


def calc_thrust(p_k, p_a, fuel, i) -> Thrust:
    props = fuel_props(fuel)
    a = int(props["al_pct"])
    P_ud_st = props["P_ud"]
    T_st = props["T"]
    R = props["R_st"]
    k = props["k_st"]

    P_ud_pr = specific_thrust_corrected(P_ud_st, a)
    P_ud_r = specific_thrust_design(P_ud_pr, p_k, p_a)
    T = combustion_temp(T_st, p_k)
    P_ud_0 = specific_thrust_ground(P_ud_pr, p_k)
    P_ud_v = specific_thrust_vacuum(P_ud_r, R, T, k, p_a, p_k)

    # Equation 1: corrected standard specific thrust
    if a == 0:
        body = f'P^"пр"_"уд {i}" = 0.96 dot {fmt(P_ud_st)} = {P_ud_pr:.2f} "с"'
    else:
        body = (
            f'P^"пр"_"уд {i}" = {fmt(P_ud_st)} dot '
            f"[1-(4.3+0.17dot {a}+0.009dot {a}^2) dot 10^(-2)]"
            f' = {P_ud_pr:.2f} "с"'
        )
    print(eq(body))

    # Equation 2: design-condition specific thrust
    body = (
        f'P^"р"_"уд {i}" = {P_ud_pr:.2f} + 19.4 + 0.76 dot {fmt(p_k)}'
        f" - 0.003 dot {fmt(p_k)}^2 - 70 dot {p_a} + 25 dot {p_a}^2"
        f' = {P_ud_r:.2f} "с"'
    )
    print(eq(body))

    # Equation 3: combustion temperature
    body = f'T_{i} = {fmt(T_st)} + 1.12 dot ({fmt(p_k)} - 40) = {T:.1f} "К"'
    print(eq(body))

    # Ground-level specific thrust: formula (3.3) at p_a = 1 bar. Only the first
    # stage needs it — it is the P_уд.01 term of (3.6) — so only that one is
    # emitted, matching the dashes in the summary table.
    if i == 1:
        body = (
            f'P_"уд.0{i}" = {P_ud_pr:.2f} + 19.4 + 0.76 dot {fmt(p_k)}'
            f" - 0.003 dot {fmt(p_k)}^2 - 70 dot 1 + 25 dot 1^2"
            f' = {P_ud_0:.2f} "с"'
        )
        print(eq(body))

    # Equation 4: vacuum specific thrust
    km1_k = f"({k}-1)/{k}"
    body = (
        f'P^"п"_"уд {i}" = {P_ud_r:.2f}'
        f" + ({fmt(R)} dot {T:.1f})/({G0}^2 dot {P_ud_r:.2f})"
        f" dot ({p_a}/{fmt(p_k)})^({km1_k})"
        f' = {P_ud_v:.2f} "с"'
    )
    print(eq(body))

    print()
    return Thrust(P_ud_pr, P_ud_r, T, P_ud_0, P_ud_v)


def calc_weights(p_k, p_a, fuel, d_m, i) -> Weight:
    props = fuel_props(fuel)
    rho_t = float(props["rho"])
    R = float(props["R_st"])
    k_gas = float(props["k_st"])
    T_st = float(props["T"])

    u, rho_u = burn_rate(fuel, p_k)  # u [mm/s], rho_u [kg/(m²·s)]
    l_z = l_coefficient(rho_u, i)
    T = combustion_temp(T_st, p_k)  # K

    K0 = k0_from_k(k_gas)
    fa_fkp = interp_chart(CHART_FA, f"k{k_gas:.2f}", p_k / p_a)

    # delta_tz from chart-3-6 (mm), converted to dimensionless K_тз = δ_тз / d_м
    delta_tz_mm = interp_chart(CHART_DTZ, A_TZ, d_m)
    K_tz = (delta_tz_mm * 1e-3) / d_m

    # (3.7) case and bottoms; η (safety factor) multiplies the mass
    eta_37 = 1.6 if i == 3 else ETA
    a = (math.pi / 2 * l_z + 1) * (p_k * 1e5 * RHO_M) / SIGMA_V * eta_37
    print(
        eq(
            f"a_{i} = (pi/2 dot {l_z:.1f} + 1)"
            f" dot ({fmt(p_k)} dot 10^5 dot {fmt(RHO_M)})"
            f" / ({SIGMA_V / 1e6:.0f} dot 10^6) dot {eta_37}"
            f' = {a:.1f} "кг/м³"'
        )
    )

    # (3.8) armor coating and adhesive; u in mm/s per formula notation
    b = (
        math.pi
        / 2
        * RHO_BR
        * (ALPHA_BR / (2 * u) * (1 - D_K_BAR**2) + l_z * (1 - EPSILON))
    )
    print(
        eq(
            f"b_{i} = pi/2 dot {fmt(RHO_BR)}"
            f" dot [{ALPHA_BR} / (2 dot {u:.2f}) dot (1 - {D_K_BAR}^2)"
            f" + {l_z:.1f} dot (1 - {EPSILON})]"
            f' = {b:.1f} "кг/м³"'
        )
    )

    # (3.9) nozzle with heat protection
    c = (
        2.03
        * rho_u
        * RHO_C_AVG
        * math.sqrt(R * T)
        / (K0 * p_k * 1e5 * math.sin(BETA_C))
        * (fa_fkp - 1)
        * l_z
        * ALPHA_C
    )
    beta_deg = int(round(math.degrees(BETA_C)))
    print(eq(f'F_("а{i}") / F_"кр" = {fa_fkp:.2f}'))
    print(
        eq(
            f"c_{i}"
            f" = (2.03 dot {rho_u:.2f} dot {fmt(RHO_C_AVG)} dot sqrt({fmt(R)} dot {T:.1f}))"
            f" / ({K0:.3f} dot {fmt(p_k)} dot 10^5 dot sin({beta_deg}°))"
            f" dot ({fa_fkp:.2f} - 1) dot {l_z:.1f} dot {ALPHA_C}"
            f' = {c:.1f} "кг/м³"'
        )
    )

    # (3.10) heat-protection thickness ratio
    print(eq(f'K_("тз"{i}) = {delta_tz_mm:.2f} dot 10^(-3) / {d_m} = {K_tz:.5f}'))

    # (3.11) heat-protection coefficient
    q = K_tz * (1.96 + math.pi * (0.37 * l_z - 0.30)) * RHO_TZ
    print(
        eq(
            f"q_{i} = {K_tz:.5f}"
            f" dot [1.96 + pi (0.37 dot {l_z:.1f} - 0.30)]"
            f" dot {fmt(RHO_TZ)}"
            f' = {q:.1f} "кг/м³"'
        )
    )

    # (3.12) propellant charge coefficient ψ
    psi = math.pi / 4 * (1 - D_K_BAR**2) * rho_t
    print(eq(f'psi_{i} = pi/4 (1 - {D_K_BAR}^2) dot {fmt(rho_t)} = {psi:.1f} "кг/м³"'))

    # (3.13) motor structural coefficient
    a_dv = (a + b + c + q) / (psi * l_z)
    print(
        eq(
            f'a_("дв{i}") = ({a:.1f} + {b:.1f} + {c:.1f} + {q:.1f})'
            f" / ({psi:.1f} dot {l_z:.1f})"
            f" = {a_dv:.4f}"
        )
    )

    print()
    return Weight(a, b, c, q, psi, l_z, a_dv)


def emit_thrust() -> tuple[list[Thrust], float]:
    """Specific-thrust section: per-stage equations, summary table, P_уд.ср."""
    section("Удельная тяга", "(3.1) - (3.6)")
    thrust = []
    for i, s in enumerate(STAGES, 1):
        stage_header(i)
        thrust.append(calc_thrust(s["p_k"], s["p_a"], s["fuel"], i))

    labels = ['$P_"уд.ст"^"пр"$, с', '$P_"уд"^"р"$, с', "$T$, К"]
    fmts = [".2f", ".2f", ".1f"]
    attrs = ["P_ud_pr", "P_ud_r", "T"]
    rows = [
        param_row(label, [getattr(t, attr) for t in thrust], spec)
        for label, spec, attr in zip(labels, fmts, attrs)
    ]
    # P_уд.0 is only used for the first stage (the P_уд.01 term of 3.6); the
    # upper stages never fire at sea level, so their cells are dashed.
    rows.append(
        param_row('$P_"уд.0"$, с', [f"${thrust[0].P_ud_0:.2f}$", "[---]", "[---]"])
    )
    rows.append(param_row('$P_"уд.п"$, с', [t.P_ud_v for t in thrust], ".2f"))
    param_table(rows)
    print()

    P_ud_01 = thrust[0].P_ud_0
    P_ud_v1, P_ud_v2, P_ud_v3 = (t.P_ud_v for t in thrust)
    P_ud_avg = (((P_ud_01 + P_ud_v1) / 2) + P_ud_v2 + P_ud_v3) / 3
    print(
        eq(
            f'P_"уд.ср" = 1/3 (({P_ud_01:.2f}+{P_ud_v1:.2f})/2'
            f"+{P_ud_v2:.2f}+{P_ud_v3:.2f})"
            f' = {P_ud_avg:.2f} "с"'
        )
    )
    print()
    return thrust, P_ud_avg


def emit_weights() -> tuple[list[Weight], list[float]]:
    """Weight-coefficient section: per-stage equations and the K_i table."""
    section("Весовые коэффициенты", "(3.7) - (3.14)")
    weight = []
    for i, s in enumerate(STAGES, 1):
        stage_header(i)
        weight.append(calc_weights(s["p_k"], s["p_a"], s["fuel"], s["d_m"], i))

    entries = [
        ("$a_i$, кг/м³", ".1f", "a"),
        ("$b_i$, кг/м³", ".1f", "b"),
        ("$c_i$, кг/м³", ".1f", "c"),
        ("$q_i$, кг/м³", ".1f", "q"),
        ("$psi_i$, кг/м³", ".1f", "psi"),
        ('$a_("дв"i)$', ".4f", "a_dv"),
    ]
    rows = [
        param_row(label, [getattr(w, attr) for w in weight], spec)
        for label, spec, attr in entries
    ]
    # α_ω — only the 3rd stage carries a guarantee fuel reserve
    a_omega_stages = [0.0, 0.0, A_OMEGA_3]
    a_omega_cells = [f"${v:.4f}$" if v else "$-$" for v in a_omega_stages]
    rows.append(param_row("$a_(omega i)$", a_omega_cells))
    # N — same tail-section coefficient for every stage
    rows.append(param_row("$N_i$", [f"${N_TAIL:.4f}$" for _ in range(3)]))
    # K — engine coefficient: a_дв plus the guarantee fuel reserve
    k_values = [weight[j].a_dv + a_omega_stages[j] for j in range(3)]
    rows.append(param_row("$K_i$", [f"${v:.4f}$" for v in k_values]))
    param_table(rows)
    print()

    print(eq(f"a_(omega 3) = {A_OMEGA_3}"))
    print(eq(f"N_i = {N_TAIL}"))
    print()
    return weight, k_values


def emit_trajectory(thrust: list[Thrust], P_ud_avg: float) -> float:
    """Active-segment section: end-of-burn parameters, V_к and relative weights."""
    section("Параметры в конце активного участка", "(3.15) - (3.18)")
    h_k = np.interp(L_FULL, TRAJ_RANGE, TRAJ_HK)
    l_k = np.interp(L_FULL, TRAJ_RANGE, TRAJ_LK)
    theta_k = np.interp(L_FULL, TRAJ_RANGE, TRAJ_THETA)
    print(
        eq(
            f'h_"к" approx {fmt(h_k)} "км", quad '
            f'l_"к" approx {fmt(l_k)} "км", quad '
            f'theta.alt_"к" approx {fmt(theta_k)} degree'
        )
    )
    print()

    # Required burnout velocity V_к: the characteristic (Tsiolkovsky) velocity
    # Σ g₀ P_уд.п ln(1/(1-μ_к)) divided by the loss factor k_V.
    mu_k = [s["mu_k"] for s in STAGES]
    p_ud_p = [t.P_ud_v for t in thrust]
    char_v = sum(G0 * p_ud_p[i] * math.log(1 / (1 - mu_k[i])) for i in range(3))
    v_k_req = char_v / K_V
    sum_body = " + ".join(
        f"{G0} dot {p_ud_p[i]:.2f} dot ln 1/(1-{fmt(mu_k[i])})" for i in range(3)
    )
    print(eq(f'V_"к" = 1/{K_V} ({sum_body}) = {v_k_req:.0f} "м/с"'))

    # Velocity demand including losses: V_к + ΔV_к = k_V·V_к.
    print(
        eq(
            f'V_"к" + Delta V_"к" = k_V V_"к"'
            f' = {K_V} dot {v_k_req:.0f} = {char_v:.0f} "м/с"'
        )
    )
    print()

    # Relative fuel weights of the subrockets (3.18): the characteristic
    # velocity k_V·V_к is split equally across the n stages.
    n = len(STAGES)
    mu_calc = 1 - math.exp(-(K_V * v_k_req) / (n * G0 * P_ud_avg))
    print(
        eq(
            f'mu_("к"i) = 1 - exp(-({K_V} dot {v_k_req:.0f})'
            f"/({n} dot {G0} dot {P_ud_avg:.2f})) = {mu_calc:.3f}"
        )
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

    u = [burn_rate(s["fuel"], s["p_k"])[0] for s in STAGES]

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

    dt = [d_m[i] * (1 - D_K_BAR) / (2 * u[i] * 1e-3) for i in range(n)]
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
        print(eq(f'm_(0 {i + 1}) = {num_src}/({denom_src}) = {m0[i]:.0f} "кг"'))
        w = weight[i]
        print(
            eq(
                f"d_(м {i + 1}) = root(3, ((1 - {N_TAIL}) dot {m0[i]:.0f} - {payload[i]:.0f})"
                f"/((1 + {w.a_dv:.4f}) dot {w.psi:.1f} dot {w.l_z:.1f}))"
                f' = {d_m[i]:.2f} "м"'
            )
        )
        print(
            eq(
                f"Delta t_(к {i + 1}) = ({d_m[i]:.2f} dot (1 - {D_K_BAR}))"
                f'/(2 dot {u[i]:.2f} dot 10^(-3)) = {dt[i]:.1f} "с"'
            )
        )
        print(
            eq(
                f"lambda_({lam_sub[i]}{i + 1}) = {dt[i]:.1f}"
                f"/({mu_k:.3f} dot {p_ud[i]:.2f}) = {lam[i]:.3f}"
            )
        )
        if i == 0:
            print(
                eq(
                    f'P_("м"1) = (4 dot {m0[0]:.0f})/(pi dot {d_m[0]:.2f}^2)'
                    f' = {p_m1:.0f} "кг/м²"'
                )
            )
        print(
            eq(
                f'omega_(з {i + 1}) = {mu_k:.3f} dot {m0[i]:.0f} = {omega_z[i]:.0f} "кг"'
            )
        )
        if i == 0:
            print(
                eq(
                    f"P_(0 1) = {thrust[0].P_ud_0:.2f} dot {G0} dot {omega_z[0]:.0f} / {dt[0]:.1f}"
                    f' = {P_r1 / 1000:.1f} "кН"'
                )
            )
            print(
                eq(
                    f'P_("п"1) = {thrust[0].P_ud_v:.2f} dot {G0} dot {omega_z[0]:.0f} / {dt[0]:.1f}'
                    f' = {P_v[0] / 1000:.1f} "кН"'
                )
            )
        else:
            print(
                eq(
                    f'P_("п"{i + 1}) = {thrust[i].P_ud_v:.2f} dot {G0} dot {omega_z[i]:.0f} / {dt[i]:.1f}'
                    f' = {P_v[i] / 1000:.1f} "кН"'
                )
            )
        print(
            eq(
                f"l_(к {i + 1}) approx 1.15 dot {weight[i].l_z:.1f} dot {d_m[i]:.2f}"
                f' = {l_k[i]:.2f} "м"'
            )
        )
        print()

    p_r1_cells = [f"${P_r1 / 1000:.1f}$", "$-$", "$-$"]
    rows = [
        param_row("$m_(0 i)$, кг", m0, ".0f"),
        param_row("$d_(м i)$, м", d_m, ".2f"),
        param_row("$Delta t_(к i)$, с", dt, ".1f"),
        param_row("$lambda_([0\\/п] i)$", lam, ".3f"),
        param_row("$omega_(з i)$, кг", omega_z, ".0f"),
        param_row("$P_(0 1)$, кН", p_r1_cells),
        param_row('$P_("п" i)$, кН', [v / 1000 for v in P_v], ".1f"),
        param_row("$l_(к i)$, м", l_k, ".2f"),
    ]
    param_table(rows)
    print()
    return Subrockets(m0=m0, omega_z=omega_z, dt=dt, d_m=d_m)


def emit_geometry(weight: list[Weight], d_m: list[float]) -> None:
    """Charge and nozzle geometry (3.28–3.42): charge length, slot height and
    burning surface (3.28–3.30); throat/exit areas and diameters (3.31–3.34);
    igniter, charge and channel diameters (3.35–3.37); nozzle convergent and
    divergent lengths, igniter/bottoms sizes and overall stage lengths
    (3.38–3.42)."""
    section("Геометрия зарядов и сопел", "(3.28) - (3.43)")
    beta_deg = int(round(math.degrees(BETA_C)))
    cot_beta = 1 / math.tan(BETA_C)

    # ---- Per-stage geometry, each stage independent ----
    geo = []
    for i, s in enumerate(STAGES):
        props = fuel_props(s["fuel"])
        R = float(props["R_st"])
        k = float(props["k_st"])
        T_st = float(props["T"])
        p_k = s["p_k"]
        p_a = s["p_a"]
        l_z = weight[i].l_z
        d = d_m[i]

        _, rho_u = burn_rate(s["fuel"], p_k)  # u·ρ_т, kg/(m²·s)
        T = combustion_temp(T_st, p_k)
        K0 = k0_from_k(k)
        fa_fkp = interp_chart(CHART_FA, f"k{k:.2f}", p_k / p_a)
        # δ_тз read at the prototype d_м, matching the weight section (3.10),
        # so the heat-protection thickness is consistent across the report.
        delta_tz_mm = interp_chart(CHART_DTZ, A_TZ, s["d_m"])

        l_zi = l_z * d  # (3.28) charge length
        h_slot = (0.37 * l_z - 0.30) * d  # (3.29) slot height
        S = K_S * l_z * d**2  # (3.30) burning surface
        d_kr2 = (4 * S * rho_u * math.sqrt(R * T)) / (
            math.pi * K0 * p_k * 1e5 * N_NOZZLES
        )  # (3.31) throat diameter squared
        d_kr = math.sqrt(d_kr2)
        F_kr = math.pi * d_kr2 / 4  # (3.32) throat area
        F_a = fa_fkp * F_kr  # (3.33) exit area
        d_a = math.sqrt(4 * F_a / math.pi)  # (3.34) exit diameter
        l_v = 0.1 * d  # (3.35) igniter length
        # Case thickness from thin-walled hoop-stress condition (matches a, 3.7)
        delta_k = ETA * p_k * 1e5 * d / (2 * SIGMA_V)
        delta_tz = delta_tz_mm * 1e-3  # heat-protection thickness, m
        d_z = d - 2 * delta_k - 2 * delta_tz  # (3.37) charge diameter
        d_k = D_K_BAR * d_z  # (3.36) channel diameter
        l_dk = (d_k - d_kr) / 2 * cot_beta  # (3.38) convergent length
        l_a = (d_a - d_kr) / 2 * cot_beta  # (3.39) divergent length
        d_v = 0.2 * d  # (3.40) igniter diameter
        l_dn = 0.3 * d  # (3.41) bottoms length
        L = l_zi + l_a + l_dk + H_RUDDER + l_v  # (3.42) stage length

        geo.append(
            {
                "R": R, "T": T, "K0": K0, "rho_u": rho_u, "fa_fkp": fa_fkp,
                "p_k": p_k, "l_z": l_z, "d": d, "l_zi": l_zi, "h_slot": h_slot,
                "S": S, "d_kr2": d_kr2, "d_kr": d_kr, "F_kr": F_kr, "F_a": F_a,
                "d_a": d_a, "l_v": l_v, "delta_k": delta_k,
                "delta_tz_mm": delta_tz_mm, "d_z": d_z, "d_k": d_k,
                "l_dk": l_dk, "l_a": l_a, "d_v": d_v, "l_dn": l_dn, "L": L,
            }
        )  # fmt: skip

    # ---- Per-stage numeric results ----
    for i, g in enumerate(geo, 1):
        stage_header(i)
        print(eq(f'l_(з {i}) = {g["l_z"]:.1f} dot {g["d"]:.2f} = {g["l_zi"]:.2f} "м"'))
        print(
            eq(
                f"h_{i} = (0.37 dot {g['l_z']:.1f} - 0.30) dot {g['d']:.2f}"
                f' = {g["h_slot"]:.3f} "м"'
            )
        )
        print(
            eq(
                f'S_{i} = {K_S} dot {g["l_z"]:.1f} dot {g["d"]:.2f}^2 = {g["S"]:.2f} "м²"'
            )
        )
        print(
            eq(
                f'd_("кр" {i})^2 = (4 dot {g["S"]:.2f} dot {g["rho_u"]:.2f} dot '
                f"sqrt({fmt(g['R'])} dot {g['T']:.1f}))"
                f"/(pi dot {g['K0']:.3f} dot {g['p_k']} dot 10^5 dot {N_NOZZLES})"
                f' = {g["d_kr2"]:.4f} "м²"'
            )
        )
        print(eq(f'd_("кр" {i}) = sqrt({g["d_kr2"]:.4f}) = {g["d_kr"]:.3f} "м"'))
        print(eq(f'F_("кр" {i}) = (pi dot {g["d_kr2"]:.4f})/4 = {g["F_kr"]:.4f} "м²"'))
        print(
            eq(
                f"F_(a {i}) = {g['fa_fkp']:.2f} dot {g['F_kr']:.4f}"
                f' = {g["F_a"]:.4f} "м²"'
            )
        )
        print(eq(f'd_(a {i}) = sqrt((4 dot {g["F_a"]:.4f})/pi) = {g["d_a"]:.3f} "м"'))
        print(eq(f'l_(в {i}) = 0.1 dot {g["d"]:.2f} = {g["l_v"]:.3f} "м"'))
        print(
            eq(
                f"delta_(к {i}) = ({ETA} dot {g['p_k']} dot 10^5 dot {g['d']:.2f})"
                f"/(2 dot {SIGMA_V / 1e6:.0f} dot 10^6)"
                f' = {g["delta_k"] * 1e3:.2f} dot 10^(-3) "м"'
            )
        )
        print(eq(f'delta_("тз" {i}) = {g["delta_tz_mm"]:.2f} dot 10^(-3) "м"'))
        print(
            eq(
                f"d_(з {i}) = {g['d']:.2f} - 2 dot {g['delta_k'] * 1e3:.2f} dot 10^(-3)"
                f" - 2 dot {g['delta_tz_mm']:.2f} dot 10^(-3)"
                f' = {g["d_z"]:.3f} "м"'
            )
        )
        print(eq(f'd_(к {i}) = {D_K_BAR} dot {g["d_z"]:.3f} = {g["d_k"]:.3f} "м"'))
        print(
            eq(
                f'l_("дк" {i}) = (({g["d_k"]:.3f} - {g["d_kr"]:.3f})/2)'
                f' dot ctg({beta_deg}°) = {g["l_dk"]:.3f} "м"'
            )
        )
        print(
            eq(
                f"l_(a {i}) = (({g['d_a']:.3f} - {g['d_kr']:.3f})/2)"
                f' dot ctg({beta_deg}°) = {g["l_a"]:.3f} "м"'
            )
        )
        print(eq(f'd_(в {i}) = 0.2 dot {g["d"]:.2f} = {g["d_v"]:.3f} "м"'))
        print(eq(f'l_("дн" {i}) approx 0.3 dot {g["d"]:.2f} = {g["l_dn"]:.3f} "м"'))
        print(
            eq(
                f"L_{i} = {g['l_zi']:.2f} + {g['l_a']:.3f} + {g['l_dk']:.3f}"
                f" + {H_RUDDER} + {g['l_v']:.3f}"
                f' = {g["L"]:.2f} "м"'
            )
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
    rows = [
        param_row(label, [g[key] for g in geo], spec) for label, spec, key in entries
    ]
    param_table(rows)
    print()

    # ---- Condition (3.43): d_м ≥ d_a (1 + √2) for 4-nozzle layout ----
    factor = 1 + math.sqrt(2)
    for i, g in enumerate(geo, 1):
        d_mi = d_m[i - 1]
        d_ai = g["d_a"]
        rhs = d_ai * factor
        verdict = "проходит" if d_mi >= rhs else "не проходит"
        print(
            eq(
                f"d_(м {i}) = {d_mi:.2f} >= d_(a {i}) (1+sqrt(2))"
                f' = {rhs:.2f} - "{verdict}"'
            )
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
    path.write_text(json.dumps(cfg, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"updated {TRAJ_CONFIG_PATH} ({len(drift)} field(s))", file=sys.stderr)


def main():
    thrust, P_ud_avg = emit_thrust()
    weight, k_values = emit_weights()
    mu_k = emit_trajectory(thrust, P_ud_avg)
    sub = emit_masses(weight, k_values, mu_k, thrust)
    emit_geometry(weight, sub.d_m)
    sync_traj_config(thrust, sub, write="--write-traj-config" in sys.argv)


if __name__ == "__main__":
    main()
