from ortools.sat.python import cp_model


class MPVRPCCModel:
    def __init__(self, instance_data):
        self.data = instance_data
        self.model = cp_model.CpModel()

        # Ensembles et Paramètres
        self.K = range(self.data['nb_trucks'])
        self.P = range(self.data['nb_products'])
        self.R = range(self.data['nb_mini_routes'])
        self.S = self.data['nodes']['stations']
        self.D = self.data['nodes']['depots']
        self.G = self.data['nodes']['garages']
        self.N = self.G + self.D + self.S

        self.C = self.data['distances']
        self.Q = self.data['truck_capacity']
        self.CC = self.data['changeover_cost']

        # Variables de décision
        self.x = {}  # Routage: x[i, j, k, r]
        self.y = {}  # Affectation produit: y[r, k, p]
        self.q = {}  # Quantité livrée: q[i, p, k, r]
        self.delta = {}  # Nettoyage: delta[k, r]

        self._create_variables()
        self._set_constraints()
        self._set_objective()

    def _create_variables(self):
        # x_ijkr: binaire
        for k in self.K:
            for r in self.R:
                for i in self.N:
                    for j in self.N:
                        if i != j:
                            self.x[i, j, k, r] = self.model.NewBoolVar(f'x_{i}_{j}_{k}_{r}')

        # y_rkp: binaire
        for k in self.K:
            for r in self.R:
                for p in self.P:
                    self.y[r, k, p] = self.model.NewBoolVar(f'y_{r}_{k}_{p}')

        # q_ipkr: entier >= 0
        for k in self.K:
            for r in self.R:
                for i in self.S:
                    for p in self.P:
                        # On limite la livraison à la demande max pour réduire l'espace de recherche
                        max_demand = self.data['demands'][str(i)][str(p)]
                        self.q[i, p, k, r] = self.model.NewIntVar(0, max_demand, f'q_{i}_{p}_{k}_{r}')

        # delta_kr: binaire
        for k in self.K:
            for r in self.R:
                self.delta[k, r] = self.model.NewBoolVar(f'delta_{k}_{r}')

    def _set_constraints(self):
        # 1. Satisfaction de la demande
        for i in self.S:
            for p in self.P:
                self.model.Add(
                    sum(self.q[i, p, k, r] for k in self.K for r in self.R) == self.data['demands'][str(i)][str(p)]
                )

        # 2. Capacité et lien avec y
        for k in self.K:
            for r in self.R:
                for p in self.P:
                    self.model.Add(
                        sum(self.q[i, p, k, r] for i in self.S) <= self.Q[k] * self.y[r, k, p]
                    )

        # 3. Mono-produit par mini-route
        for k in self.K:
            for r in self.R:
                self.model.Add(sum(self.y[r, k, p] for p in self.P) <= 1)

        # 4. Détection du changement (Nettoyage)
        for k in self.K:
            for r in range(1, len(self.R)):
                for p in self.P:
                    for pp in self.P:
                        if p != pp:
                            # delta_kr >= y_rkp + y_{(r-1)kp'} - 1
                            self.model.Add(self.delta[k, r] >= self.y[r, k, p] + self.y[r - 1, k, pp] - 1)

        # 5. Conservation de flux
        for k in self.K:
            for r in self.R:
                for i in self.N:
                    # Sortants
                    out_flow = sum(self.x[i, j, k, r] for j in self.N if (i, j, k, r) in self.x)
                    # Entrants
                    in_flow = sum(self.x[j, i, k, r] for j in self.N if (j, i, k, r) in self.x)

                    if i in self.S or i in self.D:
                        self.model.Add(out_flow == in_flow)
                    elif i in self.G:
                        # Le garage est le point de départ/arrivée de la mini-route (simplification)
                        # Pour une mini-route r, on peut imposer un départ du garage
                        pass

        # 6. Lien entre routage et livraison (si on livre à i, on doit y passer)
        for k in self.K:
            for r in self.R:
                for i in self.S:
                    # Si une livraison est faite à i, alors au moins un arc entrant à i doit être actif
                    is_visited = self.model.NewBoolVar(f'visited_{i}_{k}_{r}')
                    in_flow = sum(self.x[j, i, k, r] for j in self.N if (j, i, k, r) in self.x)
                    self.model.Add(in_flow >= is_visited)

                    for p in self.P:
                        # Si q_ipkr > 0, alors is_visited = 1
                        self.model.Add(self.q[i, p, k, r] <= self.Q[k] * is_visited)

    def _set_objective(self):
        transport_cost = sum(
            self.C[i][j] * self.x[i, j, k, r]
            for (i, j, k, r) in self.x
        )
        cleaning_cost = sum(
            self.CC * self.delta[k, r]
            for k in self.K for r in self.R
        )
        self.model.Minimize(transport_cost + cleaning_cost)

    def solve(self, timeout=60):
        solver = cp_model.CpSolver()
        solver.parameters.max_time_in_seconds = timeout
        status = solver.Solve(self.model)
        return solver, status
