import math
import os
from ortools.linear_solver import pywraplp

class MPVRPInstance:
    def __init__(self, filepath):
        self.filepath = filepath
        self.nb_products = 0
        self.nb_depots = 0
        self.nb_garages = 0
        self.nb_stations = 0
        self.nb_vehicles = 0

        self.transition_matrix = []
        self.vehicles = []
        self.depots = []
        self.garages = []
        self.stations = []
        self.nodes = []

        self._parse()
        self._compute_distances()

    def _parse(self):
        with open(self.filepath, 'r') as f:
            lines = [l.strip() for l in f.readlines() if l.strip() and not l.startswith('#')]

        dims = list(map(int, lines[0].split()))
        self.nb_products, self.nb_depots, self.nb_garages, self.nb_stations, self.nb_vehicles = dims

        current_line = 1
        for _ in range(self.nb_products):
            row = list(map(float, lines[current_line].split()))
            self.transition_matrix.append(row)
            current_line += 1

        for _ in range(self.nb_vehicles):
            parts = lines[current_line].split()
            v = {'id': int(parts[0]), 'capacity': float(parts[1]), 'garage_id': int(parts[2]), 'start_product': int(parts[3]) - 1}
            self.vehicles.append(v)
            current_line += 1

        for _ in range(self.nb_depots):
            parts = lines[current_line].split()
            stocks = list(map(float, parts[3:]))
            d = {'type': 'depot', 'id': int(parts[0]), 'x': float(parts[1]), 'y': float(parts[2]), 'stocks': stocks}
            self.depots.append(d)
            self.nodes.append(d)
            current_line += 1

        for _ in range(self.nb_garages):
            parts = lines[current_line].split()
            g = {'type': 'garage', 'id': int(parts[0]), 'x': float(parts[1]), 'y': float(parts[2])}
            self.garages.append(g)
            self.nodes.append(g)
            current_line += 1

        for _ in range(self.nb_stations):
            parts = lines[current_line].split()
            demands = list(map(float, parts[3:]))
            s = {'type': 'station', 'id': int(parts[0]), 'x': float(parts[1]), 'y': float(parts[2]), 'demands': demands}
            self.stations.append(s)
            self.nodes.append(s)
            current_line += 1

    def _compute_distances(self):
        self.dist_matrix = {}
        for i, n1 in enumerate(self.nodes):
            for j, n2 in enumerate(self.nodes):
                dist = math.sqrt((n1['x'] - n2['x'])**2 + (n1['y'] - n2['y'])**2)
                self.dist_matrix[(i, j)] = dist

class MPVRPSolver:
    def __init__(self, instance):
        self.inst = instance
        self.solver = pywraplp.Solver.CreateSolver('SCIP')
        if not self.solver: self.solver = pywraplp.Solver.CreateSolver('CBC')

        # Augmentation pour gérer les instances complexes
        self.MAX_MINI_ROUTES = 10
        self.solver.set_time_limit(300000) # 2 minutes max
        self.infinity = self.solver.infinity()

    def solve(self):
        print("🛠️ Construction du modèle (Multi-Dépôts)...")

        x = {} # Routage
        y = {} # Produit
        q = {} # Livraison Station
        load = {} # Chargement au Dépôt (NOUVEAU)
        delta = {} # Nettoyage

        all_indices = range(len(self.inst.nodes))
        depot_indices = [i for i, n in enumerate(self.inst.nodes) if n['type'] == 'depot']

        for k in range(self.inst.nb_vehicles):
            cap = self.inst.vehicles[k]['capacity']
            for r in range(self.MAX_MINI_ROUTES):
                delta[k, r] = self.solver.BoolVar(f'd_{k}_{r}')

                for p in range(self.inst.nb_products):
                    y[k, r, p] = self.solver.BoolVar(f'y_{k}_{r}_{p}')

                    # Variable de chargement spécifique par dépôt
                    for d_idx in depot_indices:
                        load[k, r, d_idx, p] = self.solver.NumVar(0, cap, f'load_{k}_{r}_{d_idx}_{p}')

                for i in all_indices:
                    for j in all_indices:
                        if i != j:
                            x[k, r, i, j] = self.solver.BoolVar(f'x_{k}_{r}_{i}_{j}')

                    if self.inst.nodes[i]['type'] == 'station':
                        for p in range(self.inst.nb_products):
                            if self.inst.nodes[i]['demands'][p] > 0:
                                q[k, r, i, p] = self.solver.NumVar(0, cap, f'q_{k}_{r}_{i}_{p}')

        print("🔗 Ajout des contraintes...")

        # 1. SATISFACTION DEMANDE
        for i in all_indices:
            node = self.inst.nodes[i]
            if node['type'] == 'station':
                for p in range(self.inst.nb_products):
                    if node['demands'][p] > 0:
                        self.solver.Add(sum(q[k, r, i, p] for k in range(self.inst.nb_vehicles) for r in range(self.MAX_MINI_ROUTES)) == node['demands'][p])

        # 2. GESTION DES DÉPÔTS (STOCK & LIEN)
        # Pour chaque dépôt réel, on vérifie son stock
        for d_idx in depot_indices:
            real_depot_idx = 0
            # Retrouver l'index dans la liste self.inst.depots
            for idx, d in enumerate(self.inst.depots):
                if d['id'] == self.inst.nodes[d_idx]['id']:
                    real_depot_idx = idx
                    break

            for p in range(self.inst.nb_products):
                # Contrainte de Stock : Somme des chargements ici <= Stock disponible
                self.solver.Add(
                    sum(load[k, r, d_idx, p] for k in range(self.inst.nb_vehicles) for r in range(self.MAX_MINI_ROUTES))
                    <= self.inst.depots[real_depot_idx]['stocks'][p]
                )

        # 3. ÉQUILIBRE CHARGEMENT / LIVRAISON
        for k in range(self.inst.nb_vehicles):
            for r in range(self.MAX_MINI_ROUTES):
                for p in range(self.inst.nb_products):
                    total_delivered = sum(q[k,r,i,p] for i in all_indices if (k,r,i,p) in q)
                    total_loaded = sum(load[k,r,d,p] for d in depot_indices)

                    # Ce qu'on livre = Ce qu'on a chargé (conservation masse)
                    self.solver.Add(total_delivered == total_loaded)

                    # On ne peut charger au dépôt D que si on sort du dépôt D
                    # load <= Capacité * Somme(x[d->j])
                    for d_idx in depot_indices:
                        outgoing_traffic = sum(x[k,r,d_idx,j] for j in all_indices if d_idx!=j)
                        self.solver.Add(load[k,r,d_idx,p] <= self.inst.vehicles[k]['capacity'] * outgoing_traffic)

        # 4. CAPACITÉ & MONO-PRODUIT
        for k in range(self.inst.nb_vehicles):
            for r in range(self.MAX_MINI_ROUTES):
                cap = self.inst.vehicles[k]['capacity']
                self.solver.Add(sum(y[k, r, p] for p in range(self.inst.nb_products)) <= 1)

                for p in range(self.inst.nb_products):
                    # q <= Cap * y
                    total_delivered = sum(q[k,r,i,p] for i in all_indices if (k,r,i,p) in q)
                    self.solver.Add(total_delivered <= cap * y[k, r, p])

                    # Visite obligatoire si livraison
                    for i in all_indices:
                        if (k,r,i,p) in q:
                            incoming = sum(x[k, r, j, i] for j in all_indices if i!=j)
                            self.solver.Add(q[k, r, i, p] <= cap * incoming)

        # 5. FLUX
        for k in range(self.inst.nb_vehicles):
            for r in range(self.MAX_MINI_ROUTES):
                for i in all_indices:
                    if self.inst.nodes[i]['type'] == 'garage': continue
                    sum_in = sum(x[k, r, j, i] for j in all_indices if i != j)
                    sum_out = sum(x[k, r, i, j] for j in all_indices if i != j)
                    self.solver.Add(sum_in == sum_out)

        # 6. NETTOYAGE
        for k in range(self.inst.nb_vehicles):
            p_start = self.inst.vehicles[k]['start_product']
            for p in range(self.inst.nb_products):
                if p != p_start: self.solver.Add(delta[k, 0] >= y[k, 0, p])
            for r in range(1, self.MAX_MINI_ROUTES):
                for p1 in range(self.inst.nb_products):
                    for p2 in range(self.inst.nb_products):
                        if p1 != p2: self.solver.Add(delta[k, r] >= y[k, r-1, p1] + y[k, r, p2] - 1)

        # OBJECTIF
        dist_obj = sum(self.inst.dist_matrix[i,j] * x[k,r,i,j] for k in range(self.inst.nb_vehicles) for r in range(self.MAX_MINI_ROUTES) for i in all_indices for j in all_indices if i!=j)
        clean_obj = sum(10.0 * delta[k,r] for k in range(self.inst.nb_vehicles) for r in range(self.MAX_MINI_ROUTES)) # Poids arbitraire

        self.solver.Minimize(dist_obj + clean_obj)

        print("⏳ Résolution en cours...")
        status = self.solver.Solve()

        if status in [pywraplp.Solver.OPTIMAL, pywraplp.Solver.FEASIBLE]:
            print(f"✅ Solution trouvée !")
            return self._extract_solution(x, y, q, load) # On passe 'load' en plus
        else:
            print("❌ Infeasible.")
            return None

    def _extract_solution(self, x, y, q, load):
        output_lines = []
        grand_total_dist = 0.0
        grand_total_cost = 0.0
        grand_total_changes = 0
        nb_vehicles_used = 0

        # Helpers
        def get_specific_node(nid, ntype):
            lst = self.inst.garages if ntype=='G' else (self.inst.depots if ntype=='D' else self.inst.stations)
            for n in lst:
                if n['id'] == nid: return n
            return None
        def calc_dist(n1, n2): return math.sqrt((n1['x'] - n2['x'])**2 + (n1['y'] - n2['y'])**2)
        def get_q(k, r, i, p): return q[k, r, i, p].solution_value() if (k,r,i,p) in q else 0.0
        def get_load(k, r, d, p): return load[k, r, d, p].solution_value() if (k,r,d,p) in load else 0.0

        depot_indices = [i for i, n in enumerate(self.inst.nodes) if n['type'] == 'depot']

        for k in range(self.inst.nb_vehicles):
            vehicle = self.inst.vehicles[k]
            garage_node = get_specific_node(vehicle['garage_id'], 'G')

            visited_sequence = []
            visited_sequence.append({'node': garage_node, 'type': 'G', 'qty': 0, 'prod': vehicle['start_product']})

            used_vehicle = False
            prev_prod = vehicle['start_product']

            for r in range(self.MAX_MINI_ROUTES):
                active_p = -1
                for p in range(self.inst.nb_products):
                    if y[k, r, p].solution_value() > 0.5:
                        active_p = p
                        break
                if active_p == -1: continue
                used_vehicle = True

                # Identifier quel dépôt a fourni la marchandise
                used_depot_node = None
                load_qty = 0
                for d_idx in depot_indices:
                    val = get_load(k, r, d_idx, active_p)
                    if val > 0.1:
                        load_qty = int(round(val))
                        used_depot_node = self.inst.nodes[d_idx]
                        break

                # Si on a chargé (cas normal), on ajoute le dépôt
                if used_depot_node and load_qty > 0:
                     visited_sequence.append({'node': used_depot_node, 'type': 'D', 'qty': load_qty, 'prod': active_p})

                # Identifier les stations
                for i in range(len(self.inst.nodes)):
                    if self.inst.nodes[i]['type'] == 'station':
                        qty = get_q(k, r, i, active_p)
                        if qty > 0.1:
                            qty_int = int(round(qty))
                            visited_sequence.append({'node': self.inst.nodes[i], 'type': 'S', 'qty': qty_int, 'prod': active_p})

                prev_prod = active_p

            visited_sequence.append({'node': garage_node, 'type': 'G', 'qty': 0, 'prod': prev_prod})

            # Comptage strict : Un véhicule n'est "utilisé" que s'il a visité autre chose que G->G
            if len(visited_sequence) > 2:
                nb_vehicles_used += 1
            else:
                # Si le véhicule n'a rien fait, on reset la séquence pour l'affichage propre
                visited_sequence = [{'node': garage_node, 'type': 'G', 'qty': 0, 'prod': vehicle['start_product']},
                                    {'node': garage_node, 'type': 'G', 'qty': 0, 'prod': vehicle['start_product']}]

            # Génération Lignes
            line1_parts, line2_parts = [], []
            veh_dist, veh_cost = 0.0, 0.0

            line1_parts.append(f"{vehicle['garage_id']}")
            line2_parts.append(f"{vehicle['start_product']}(0.0)")

            curr_prod = vehicle['start_product']

            for idx in range(1, len(visited_sequence)):
                curr = visited_sequence[idx]
                prev = visited_sequence[idx-1]

                veh_dist += calc_dist(prev['node'], curr['node'])

                txt = f"{curr['node']['id']}"
                if curr['type'] in ['D', 'S']:
                    braces = "[]" if curr['type'] == 'D' else "()"
                    txt += f" {braces[0]}{curr['qty']}{braces[1]}"
                line1_parts.append(txt)

                step_cost = 0.0
                if curr['prod'] != curr_prod:
                    step_cost = self.inst.transition_matrix[curr_prod][curr['prod']]
                    veh_cost += step_cost
                    grand_total_changes += 1
                    curr_prod = curr['prod']
                line2_parts.append(f"{curr['prod']}({step_cost:.1f})")

            grand_total_dist += veh_dist
            grand_total_cost += veh_cost

            output_lines.append(f"{vehicle['id']}: " + " - ".join(line1_parts))
            output_lines.append(f"{vehicle['id']}: " + " - ".join(line2_parts))
            output_lines.append("")

        output_lines.append(f"{nb_vehicles_used}")
        output_lines.append(f"{grand_total_changes}")
        output_lines.append(f"{grand_total_cost:.2f}")
        output_lines.append(f"{grand_total_dist:.2f}")
        output_lines.append("OR-Tools-SCIP")
        output_lines.append("2.5")

        return "\n".join(output_lines)
"""
if __name__ == "__main__":
    file_path = "MPVRP_S_002_s10_d2_p3.dat" # Change le nom ici pour chaque instance
    try:
        inst = MPVRPInstance(file_path)
        print(f"Chargement: {file_path}")
        solver = MPVRPSolver(inst)
        res = solver.solve()
        if res:
            base = os.path.basename(file_path).replace(".txt", "")
            with open("Sol_" + base, "w") as f: f.write(res)
            print(f"💾 Sauvegardé: Sol_{base}")
    except Exception as e: print(e)"""