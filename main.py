import math
from typing import NamedTuple

import numpy as np

from typst import eq, fmt, param_row, param_table, section
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
    specific_thrust_vacuum,
)

# Primary design requirement: full flight range L (km).
L_FULL = 12000

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
    {"p_k": 35, "p_a": 0.23, "fuel": "polyurethane", "d_m": 1.1, "mu_k": 0.65},
    {"p_k": 35, "p_a": 0.14, "fuel": "polyurethane", "d_m": 0.9, "mu_k": 0.65},
]

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

CHART_FA = "assets/chart-3-5-fa-fkp-pa-pk.csv"
CHART_DTZ = "assets/chart-3-6-delta-tz-d-m.csv"


class Thrust(NamedTuple):
    """Per-stage specific-thrust results (seconds, except T in kelvin)."""

    P_ud_pr: float  # corrected standard specific thrust
    P_ud_r: float  # design-condition specific thrust
    T: float  # combustion temperature, K
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
    return Thrust(P_ud_pr, P_ud_r, T, P_ud_v)


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
    a = (math.pi / 2 * l_z + 1) * (p_k * 1e5 * RHO_M) / SIGMA_V * ETA
    print(
        eq(
            f"a_{i} = (pi/2 dot {l_z:.1f} + 1)"
            f" dot ({fmt(p_k)} dot 10^5 dot {fmt(RHO_M)})"
            f" / ({SIGMA_V / 1e6:.0f} dot 10^6) dot {ETA}"
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
    section("Удельная тяга")
    thrust = [
        calc_thrust(s["p_k"], s["p_a"], s["fuel"], i) for i, s in enumerate(STAGES, 1)
    ]

    labels = ['$P_"уд.ст"^"пр"$, с', '$P_"уд"^"р"$, с', "$T$, К", '$P_"уд.п"$, с']
    fmts = [".2f", ".2f", ".1f", ".2f"]
    attrs = ["P_ud_pr", "P_ud_r", "T", "P_ud_v"]
    rows = [
        param_row(label, [getattr(t, attr) for t in thrust], spec)
        for label, spec, attr in zip(labels, fmts, attrs)
    ]
    param_table(rows)
    print()

    P_ud_r1 = thrust[0].P_ud_r
    P_ud_v1, P_ud_v2, P_ud_v3 = (t.P_ud_v for t in thrust)
    P_ud_avg = (((P_ud_r1 + P_ud_v1) / 2) + P_ud_v2 + P_ud_v3) / 3
    print(
        eq(
            f'P_"уд.ср" = 1/3 (({P_ud_r1:.2f}+{P_ud_v1:.2f})/2'
            f"+{P_ud_v2:.2f}+{P_ud_v3:.2f})"
            f' = {P_ud_avg:.2f} "с"'
        )
    )
    print()
    return thrust, P_ud_avg


def emit_weights() -> None:
    """Weight-coefficient section: per-stage equations and the K_i table."""
    section("Весовые коэффициенты")
    weight = [
        calc_weights(s["p_k"], s["p_a"], s["fuel"], s["d_m"], i)
        for i, s in enumerate(STAGES, 1)
    ]

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
    k_cells = [f"${weight[j].a_dv + a_omega_stages[j]:.4f}$" for j in range(3)]
    rows.append(param_row("$K_i$", k_cells))
    param_table(rows)
    print()

    print(eq(f"a_(omega 3) = {A_OMEGA_3}"))
    print(eq(f"N_i = {N_TAIL}"))
    print()


def emit_trajectory(thrust: list[Thrust], P_ud_avg: float) -> None:
    """Active-segment section: end-of-burn parameters, V_к and relative weights."""
    section("Параметры в конце активного участка")
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


def main():
    thrust, P_ud_avg = emit_thrust()
    emit_weights()
    emit_trajectory(thrust, P_ud_avg)


if __name__ == "__main__":
    main()
