import sys
import pulp

# =====================================================
# 1. Lecture simplifiée de l'instance (robuste)
# =====================================================
def read_instance(path):
    with open(path, 'r') as f:
        lines = [l.strip() for l in f if l.strip() and not l.startswith("#")]

    # -------- Header --------
    header = list(map(int, lines[0].split()))
    nb_vehicles, nb_products, nb_garages, nb_depots, nb_clients = header

    idx = 1

    # -------- Lecture brute des distances --------
    dist = []
    while idx < len(lines):
        parts = lines[idx].split()
        try:
            row = list(map(float, parts))
            dist.append(row)
            idx += 1
        except ValueError:
            break

    # -------- CORRECTION CRITIQUE : rendre dist carrée --------
    N = len(dist)
    for i in range(N):
        if len(dist[i]) < N:
            dist[i].extend([0.0] * (N - len(dist[i])))
        elif len(dist[i]) > N:
            dist[i] = dist[i][:N]

    # -------- Ensembles --------
    V = range(N)          # noeuds (0 = dépôt)
    S = range(1, N)       # stations
    P = range(nb_products)
    K = range(nb_vehicles)

    # -------- Données minimales (académiques) --------
    d = {(s, p): 1 for s in S for p in P}     # demandes fictives
    Q = {k: 10_000 for k in K}                # capacité camions
    cost_change = 100                         # coût changement produit

    return {
        "V": V,
        "S": S,
        "P": P,
        "K": K,
        "dist": dist,
        "d": d,
        "Q": Q,
        "cost_change": cost_change,
        "nb_vehicles": nb_vehicles,
        "nb_clients": len(S)
    }

# =====================================================
# 2. Modèle MPVRP-CC
# =====================================================
def build_model(data):
    model = pulp.LpProblem("MPVRP_CC", pulp.LpMinimize)

    V = data["V"]
    S = data["S"]
    P = data["P"]
    K = data["K"]
    dist = data["dist"]
    d = data["d"]
    Q = data["Q"]
    cost_change = data["cost_change"]

    # ================= VARIABLES =================

    # x_kij : camion k va de i à j
    x = pulp.LpVariable.dicts("x", (K, V, V), 0, 1, pulp.LpBinary)

    # y_ksp : camion k livre produit p à station s
    y = pulp.LpVariable.dicts("y", (K, S, P), 0, 1, pulp.LpBinary)

    # z_kp : camion k configuré pour produit p
    z = pulp.LpVariable.dicts("z", (K, P), 0, 1, pulp.LpBinary)

    # c_k : changement de produit camion k
    c = pulp.LpVariable.dicts("c", K, 0, 1, pulp.LpBinary)

    # ================= OBJECTIF =================

    model += (
        pulp.lpSum(
            dist[i][j] * x[k][i][j]
            for k in K for i in V for j in V if i != j
        )
        +
        pulp.lpSum(cost_change * c[k] for k in K)
    )

    # ================= CONTRAINTES =================

    # 1️⃣ Satisfaction de la demande
    for s in S:
        for p in P:
            model += pulp.lpSum(y[k][s][p] for k in K) == 1

    # 2️⃣ Capacité camion
    for k in K:
        model += pulp.lpSum(d[s, p] * y[k][s][p] for s in S for p in P) <= Q[k]

    # 3️⃣ Flux / retour dépôt
    for k in K:
        model += (
            pulp.lpSum(x[k][0][j] for j in V if j != 0)
            ==
            pulp.lpSum(x[k][i][0] for i in V if i != 0)
        )

    # conservation flux
    for k in K:
        for i in V:
            model += (
                pulp.lpSum(x[k][i][j] for j in V if j != i)
                ==
                pulp.lpSum(x[k][j][i] for j in V if j != i)
            )

    # 4️⃣ Un seul produit par camion
    for k in K:
        model += pulp.lpSum(z[k][p] for p in P) == 1

    # liaison y → z
    for k in K:
        for s in S:
            for p in P:
                model += y[k][s][p] <= z[k][p]

    # changement produit (simplifié)
    for k in K:
        model += c[k] >= 0

    return model, x, y, z, c

# =====================================================
# 3. Résolution
# =====================================================
def solve(model):
    status = model.solve(pulp.PULP_CBC_CMD(msg=1))
    return pulp.LpStatus[status], pulp.value(model.objective)

# =====================================================
# 4. Écriture solution (structure API)
# =====================================================
def write_solution(path, status, cost, data, x, y, z):
    with open(path, "w") as f:
        f.write(f"STATUS {status}\n")
        f.write(f"TOTAL_COST {int(cost)}\n")
        f.write(f"TOTAL_DISTANCE {int(cost)}\n")
        f.write(f"NB_VEHICLES {data['nb_vehicles']}\n")
        f.write(f"NB_CLIENTS {data['nb_clients']}\n\n")

        for k in data["K"]:
            f.write(f"VEHICLE {k}\n")

            # ROUTE (simple, sans sous-tours)
            route = [0]
            for i in data["V"]:
                for j in data["V"]:
                    if i != j and pulp.value(x[k][i][j]) == 1:
                        route.append(j)
            if route[-1] != 0:
                route.append(0)

            f.write("ROUTE " + " ".join(map(str, route)) + "\n")

            # PRODUIT
            for p in data["P"]:
                if pulp.value(z[k][p]) == 1:
                    f.write(f"PRODUCT {p}\n")

            # LIVRAISONS
            for s in data["S"]:
                for p in data["P"]:
                    if pulp.value(y[k][s][p]) == 1:
                        f.write(f"DELIVER {s} {p}\n")

            f.write("\n")

# =====================================================
# 5. Main
# =====================================================
if __name__ == "__main__":
    instance_file = sys.argv[1]
    solution_file = sys.argv[2]

    data = read_instance(instance_file)
    model, x, y, z, c = build_model(data)
    status, cost = solve(model)
    write_solution(solution_file, status, cost, data, x, y, z)

    print("Status:", status)
    print("Cost:", cost)
