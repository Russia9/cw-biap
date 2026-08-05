from utils import alpha_dv, burn_rate, l_coefficient

P_KS = [50, 35, 35]
FUELS = ["polyurethane", "polybutadiene"]


def main():
    # Pre-compute burn rates for both tables, indexed by stage (0-based)
    rates = [{fuel: burn_rate(fuel, p_k) for fuel in FUELS} for p_k in P_KS]

    # Table 1: burn rates
    print("$i$, $p_k^i$, " + ", ".join("$u$, $u dot rho_t$" for _ in FUELS))
    for i, (p_k, rate) in enumerate(zip(P_KS, rates), 1):
        row = [f"${i}$", f"${p_k}$"]
        for fuel in FUELS:
            u, rho_u = rate[fuel]
            row += [f"${u:.2f}$", f"${rho_u:.2f}$"]
        print(", ".join(row) + ",")

    print()

    # Table 2: l_z and alpha_dv
    print("$i$, " + ", ".join('$l_з^i$, $alpha_"дв"$' for _ in FUELS))
    for i, rate in enumerate(rates, 1):
        row = [f"${i}$"]
        for fuel in FUELS:
            _, rho_u = rate[fuel]
            lz = l_coefficient(rho_u, i)
            alpha = alpha_dv(rho_u, lz)
            row += [f"${lz:.1f}$", f"${alpha:.3f}$"]
        print(", ".join(row) + ",")


if __name__ == "__main__":
    main()
