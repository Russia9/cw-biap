from utils import (
    G0,
    combustion_temp,
    fuel_props,
    specific_thrust_corrected,
    specific_thrust_design,
    specific_thrust_vacuum,
)

STAGES = [
    {"p_k": 50, "p_a": 0.70, "fuel": "polybutadiene"},
    {"p_k": 35, "p_a": 0.23, "fuel": "polyurethane"},
    {"p_k": 35, "p_a": 0.14, "fuel": "polyurethane"},
]


def eq(body):
    return f"#math.equation(numbering: none, block: true, $ {body} $)"


def fmt(x):
    """Format a number: integer if whole, else up to 2 decimal places."""
    return str(int(x)) if x == int(x) else f"{x:g}"


def calc_stage(p_k, p_a, fuel, i):
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
    return P_ud_pr, P_ud_r, T, P_ud_v


def main():
    rows = [calc_stage(**s, i=i) for i, s in enumerate(STAGES, 1)]

    print("  table(")
    print("    columns: 4,")
    print("    table.header([Параметр], [I ступень], [II ступень], [III ступень]),")

    labels = [
        '$P_"уд.ст"^"пр"$, с',
        '$P_"уд"^"р"$, с',
        "$T$, К",
        '$P_"уд.п"$, с',
    ]
    fmts = [".2f", ".2f", ".1f", ".2f"]

    for label, fmt_str, col in zip(labels, fmts, range(4)):
        vals = ", ".join(f"${rows[i][col]:{fmt_str}}$" for i in range(3))
        print(f"    [{label}], {vals},")

    print("  ),")
    print()

    P_ud_r1 = rows[0][1]
    P_ud_v1, P_ud_v2, P_ud_v3 = rows[0][3], rows[1][3], rows[2][3]
    P_ud_avg = (((P_ud_r1 + P_ud_v1) / 2) + P_ud_v2 + P_ud_v3) / 3

    body = (
        f'P_"уд.ср" = 1/3 (({P_ud_r1:.2f}+{P_ud_v1:.2f})/2'
        f"+{P_ud_v2:.2f}+{P_ud_v3:.2f})"
        f' = {P_ud_avg:.2f} "с"'
    )
    print(eq(body))


if __name__ == "__main__":
    main()
