from utils import alpha_dv, burn_rate, l_coefficient

# Initial parameters
L_max = 12000  # km, maximum launch distance
m_bc = 500  # kg, payload
m_au = 120  # kg, control system mass

P_KS = [50, 35, 35]
FUELS = ["polyurethane", "polybutadiene"]


def main():
    # Pre-compute burn rates for both tables
    rates: dict[int, dict[str, tuple[float, float]]] = {}
    for i, p_k in enumerate(P_KS, 1):
        rates[i] = {fuel: burn_rate(fuel, p_k) for fuel in FUELS}

    # Table 1: burn rates
    print("$i$, $p_k^i$, " + ", ".join(f"$u$, $u dot rho_t$" for _ in FUELS))
    for i, p_k in enumerate(P_KS, 1):
        row = [f"${i}$", f"${p_k}$"]
        for fuel in FUELS:
            u, rho_u = rates[i][fuel]
            row += [f"${u:.2f}$", f"${rho_u:.2f}$"]
        print(", ".join(row) + ",")

    print()

    # Table 2: l_z and alpha_dv
    print("$i$, " + ", ".join(f'$l_з^i$, $alpha_"дв"$' for _ in FUELS))
    for i in range(1, len(P_KS) + 1):
        row = [f"${i}$"]
        for fuel in FUELS:
            _, rho_u = rates[i][fuel]
            lz = l_coefficient(rho_u, i)
            alpha = alpha_dv(rho_u, lz)
            row += [f"${lz:.1f}$", f"${alpha:.3f}$"]
        print(", ".join(row) + ",")


if __name__ == "__main__":
    main()
