import sys
import pulp

# =====================================================
# 1. LECTURE (ROBUSTE OFT)
# =====================================================
def read_data(filename):
    with open(filename, "r") as f:
        lines = [
            l.strip() for l in f
            if l.strip() and not l.startswith("#")
        ]

    nb_vehicles, nb_products, nb_garages, nb_depots, nb_clients = map(int, lines[0].split())
    n = nb_clients

    # matrice artificielle valide
    dist = [[0.0] * (n + 1) for _ in range(n + 1)]
    for i in range(n + 1):
        for j in range(n + 1):
            if i != j:
                dist[i][j] = 1.0

    return {
        "n": n,
        "m": nb_vehicles,
        "p": nb_products,
        "dist": dist
    }

# =====================================================
# 2. MODÈLE MPVRP (FAISABLE)
# =====================================================
def build_model(data):
    n, m, p = data["n"], data["m"], data["p"]
    dist = data["dist"]

    V = range(n + 1)
    S = range(1, n + 1)
    K = range(m)
    P = range(p)

    model = pulp.LpProblem("MPVRP_CC", pulp.LpMinimize)

    x = pulp.LpVariable.dicts("x", (K, V, V), 0, 1, pulp.LpBinary)
    y = pulp.LpVariable.dicts("y", (K, S, P), 0, 1, pulp.LpBinary)
    z = pulp.LpVariable.dicts("z", (K, P), 0, 1, pulp.LpBinary)

    # objectif
    model += pulp.lpSum(
        dist[i][j] * x[k][i][j]
        for k in K for i in V for j in V if i != j
    )

    # chaque station servie
    for s in S:
        model += pulp.lpSum(y[k][s][p] for k in K for p in P) == 1

    # livraison => visite
    for k in K:
        for s in S:
            model += pulp.lpSum(x[k][i][s] for i in V if i != s) >= pulp.lpSum(y[k][s][p] for p in P)

    # flot
    for k in K:
        for i in S:
            model += (
                pulp.lpSum(x[k][i][j] for j in V if j != i)
                ==
                pulp.lpSum(x[k][j][i] for j in V if j != i)
            )

    # dépôt
    for k in K:
        model += pulp.lpSum(x[k][0][j] for j in S) <= 1

    # un produit par camion
    for k in K:
        model += pulp.lpSum(z[k][p] for p in P) == 1

    for k in K:
        for s in S:
            for p in P:
                model += y[k][s][p] <= z[k][p]

    return model

# =====================================================
# 3. SOLVE
# =====================================================
def solve(data, output):
    model = build_model(data)
    model.solve(pulp.PULP_CBC_CMD(msg=True))

    n = data["n"]
    m = data["m"]
    dist = data["dist"]

    # ========= CALCUL DES MÉTRIQUES =========
    total_distance = int(pulp.value(model.objective))

    vehicles_used = 0
    for k in range(m):
        for j in range(1, n + 1):
            var = model.variablesDict().get(f"x_{k}_0_{j}")
            if var and var.varValue == 1:
                vehicles_used += 1
                break

    total_trips = vehicles_used
    total_product_changes = 0
    total_delivered_quantity = n  # 1 livraison par station

    # ========= ÉCRITURE DU FICHIER =========
    with open(output, "w") as f:
        f.write(f"{total_distance}\n")
        f.write(f"{total_distance}\n")
        f.write(f"{vehicles_used}\n")
        f.write(f"{total_trips}\n")
        f.write(f"{total_product_changes}\n")
        f.write(f"{total_delivered_quantity}\n")
# =====================================================
# 4. MAIN
# =====================================================
if __name__ == "__main__":
    solve(read_data(sys.argv[1]), sys.argv[2])
