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

        self.MAX_MINI_ROUTES = 5
        self.solver.set_time_limit(120000) # 2 minutes
        self.infinity = self.solver.infinity()

    def solve(self):
        print("🛠️ Construction du modèle (MTZ + Contrainte Garage)...")

        x = {} # Routage x[k, r, i, j]
        y = {} # Produit y[k, r, p]
        q = {} # Livraison q[k, r, i, p]
        load = {} # Chargement load[k, r, d, p]
        delta = {} # Nettoyage
        u = {} # MTZ : Ordre de passage

        all_indices = range(len(self.inst.nodes))
        depot_indices = [i for i, n in enumerate(self.inst.nodes) if n['type'] == 'depot']
        station_indices = [i for i, n in enumerate(self.inst.nodes) if n['type'] == 'station']

        for k in range(self.inst.nb_vehicles):
            cap = self.inst.vehicles[k]['capacity']
            for r in range(self.MAX_MINI_ROUTES):
                delta[k, r] = self.solver.BoolVar(f'd_{k}_{r}')
                for p in range(self.inst.nb_products):
                    y[k, r, p] = self.solver.BoolVar(f'y_{k}_{r}_{p}')
                    for d_idx in depot_indices:
                        load[k, r, d_idx, p] = self.solver.NumVar(0, cap, f'load_{k}_{r}_{d_idx}_{p}')

                for i in all_indices:
                    u[k, r, i] = self.solver.NumVar(0, len(all_indices), f'u_{k}_{r}_{i}')
                    for j in all_indices:
                        if i != j:
                            x[k, r, i, j] = self.solver.BoolVar(f'x_{k}_{r}_{i}_{j}')
                    
                    if self.inst.nodes[i]['type'] == 'station':
                        for p in range(self.inst.nb_products):
                            if self.inst.nodes[i]['demands'][p] > 0:
                                q[k, r, i, p] = self.solver.NumVar(0, cap, f'q_{k}_{r}_{i}_{p}')

        print("🔗 Ajout des contraintes...")

        # 1. SATISFACTION DEMANDE
        for i in station_indices:
            node = self.inst.nodes[i]
            for p in range(self.inst.nb_products):
                if node['demands'][p] > 0:
                    self.solver.Add(sum(q[k, r, i, p] for k in range(self.inst.nb_vehicles) for r in range(self.MAX_MINI_ROUTES)) == node['demands'][p])

        # 2. STOCKS DÉPÔTS
        for d_idx in depot_indices:
            real_depot_idx = next(idx for idx, d in enumerate(self.inst.depots) if d['id'] == self.inst.nodes[d_idx]['id'])
            for p in range(self.inst.nb_products):
                self.solver.Add(sum(load[k, r, d_idx, p] for k in range(self.inst.nb_vehicles) for r in range(self.MAX_MINI_ROUTES)) <= self.inst.depots[real_depot_idx]['stocks'][p])

        # 3. FLUX, MTZ ET ANCRAGE GARAGE
        for k in range(self.inst.nb_vehicles):
            v_info = self.inst.vehicles[k]
            # Trouver l'index local du garage assigné au véhicule
            g_idx = next(i for i, n in enumerate(self.inst.nodes) if n['type'] == 'garage' and n['id'] == v_info['garage_id'])
            
            for r in range(self.MAX_MINI_ROUTES):
                route_active = self.solver.Sum(y[k, r, p] for p in range(self.inst.nb_products))
                
                # Sortie et Entrée obligatoire au garage spécifique
                self.solver.Add(sum(x[k, r, g_idx, j] for j in all_indices if j != g_idx) == route_active)
                self.solver.Add(sum(x[k, r, i, g_idx] for i in all_indices if i != g_idx) == route_active)

                for i in all_indices:
                    # Équilibre du flux
                    sum_in = sum(x[k, r, j, i] for j in all_indices if i != j)
                    sum_out = sum(x[k, r, i, j] for j in all_indices if i != j)
                    self.solver.Add(sum_in == sum_out)
                    
                    # Interdire les autres garages
                    if self.inst.nodes[i]['type'] == 'garage' and i != g_idx:
                        self.solver.Add(sum_out == 0)

                    # MTZ - Sous-tours
                    for j in all_indices:
                        if i != j and j != g_idx:
                            self.solver.Add(u[k, r, j] >= u[k, r, i] + 1 - 100 * (1 - x[k, r, i, j]))

                # Logique de chargement vs Livraison
                for p in range(self.inst.nb_products):
                    total_delivered = sum(q[k, r, i, p] for i in station_indices if (k, r, i, p) in q)
                    total_loaded = sum(load[k, r, d, p] for d in depot_indices)
                    self.solver.Add(total_delivered == total_loaded)
                    self.solver.Add(total_delivered <= v_info['capacity'] * y[k, r, p])

                    for d_idx in depot_indices:
                        self.solver.Add(load[k, r, d_idx, p] <= v_info['capacity'] * sum(x[k, r, d_idx, j] for j in all_indices if d_idx != j))

                    for i in station_indices:
                        if (k, r, i, p) in q:
                            self.solver.Add(q[k, r, i, p] <= v_info['capacity'] * sum(x[k, r, j, i] for j in all_indices if i != j))

        # 4. NETTOYAGE (Transitions de produits)
        for k in range(self.inst.nb_vehicles):
            p_start = self.inst.vehicles[k]['start_product']
            # Première mini-route
            for p in range(self.inst.nb_products):
                if p != p_start: self.solver.Add(delta[k, 0] >= y[k, 0, p])
            # Entre mini-routes
            for r in range(1, self.MAX_MINI_ROUTES):
                for p1 in range(self.inst.nb_products):
                    for p2 in range(self.inst.nb_products):
                        if p1 != p2: self.solver.Add(delta[k, r] >= y[k, r-1, p1] + y[k, r, p2] - 1)

        # OBJECTIF
        dist_obj = sum(self.inst.dist_matrix[i, j] * x[k, r, i, j] for k in range(self.inst.nb_vehicles) for r in range(self.MAX_MINI_ROUTES) for i in all_indices for j in all_indices if i != j)
        clean_obj = sum(100.0 * delta[k, r] for k in range(self.inst.nb_vehicles) for r in range(self.MAX_MINI_ROUTES))

        self.solver.Minimize(dist_obj + clean_obj)

        print("⏳ Résolution en cours...")
        status = self.solver.Solve()

        if status in [pywraplp.Solver.OPTIMAL, pywraplp.Solver.FEASIBLE]:
            print(f"✅ Solution trouvée !")
            return self._extract_solution(x, y, q, load)
        else:
            print("❌ Aucune solution.")
            return None

    def _extract_solution(self, x, y, q, load):
        output = []
        g_total_dist = 0.0
        g_total_cost = 0.0
        g_total_changes = 0
        v_used_count = 0

        for k in range(self.inst.nb_vehicles):
            v_info = self.inst.vehicles[k]
            g_idx = next(i for i, n in enumerate(self.inst.nodes) if n['type'] == 'garage' and n['id'] == v_info['garage_id'])
            
            v_dist, v_cost = 0.0, 0.0
            full_path = [] # Liste de dict: {'node': node_obj, 'qty': q, 'prod': p}
            curr_prod = v_info['start_product']
            vehicle_is_active = False

            for r in range(self.MAX_MINI_ROUTES):
                active_p = -1
                for p in range(self.inst.nb_products):
                    if y[k, r, p].solution_value() > 0.5:
                        active_p = p
                        break
                
                if active_p == -1: continue
                if not vehicle_is_active: 
                    vehicle_is_active = True
                    v_used_count += 1

                # Reconstruire le chemin de la mini-route r en suivant x[i,j]
                current_node_idx = g_idx
                path_found = False
                
                # Sécurité pour éviter boucle infinie en cas de split de flux résiduel
                visited_in_r = 0 
                while visited_in_r < len(self.inst.nodes):
                    next_node_idx = -1
                    for j in range(len(self.inst.nodes)):
                        if current_node_idx != j and x[k, r, current_node_idx, j].solution_value() > 0.5:
                            next_node_idx = j
                            break
                    
                    if next_node_idx == -1 or next_node_idx == g_idx:
                        break # Fin de la boucle ou retour au garage
                    
                    # Infos sur le nœud suivant
                    node = self.inst.nodes[next_node_idx]
                    qty = 0
                    if node['type'] == 'depot':
                        qty = load[k, r, next_node_idx, active_p].solution_value()
                    elif node['type'] == 'station':
                        qty = q[k, r, next_node_idx, active_p].solution_value() if (k,r,next_node_idx,active_p) in q else 0
                    
                    full_path.append({'node': node, 'qty': int(round(qty)), 'prod': active_p})
                    v_dist += self.inst.dist_matrix[(current_node_idx, next_node_idx)]
                    current_node_idx = next_node_idx
                    visited_in_r += 1

                # Calculer coût de nettoyage
                if active_p != curr_prod:
                    v_cost += self.inst.transition_matrix[curr_prod][active_p]
                    g_total_changes += 1
                    curr_prod = active_p
                
                # Distance retour au garage depuis le dernier point de la mini-route
                v_dist += self.inst.dist_matrix[(current_node_idx, g_idx)]

            # Formater les lignes du véhicule
            l1 = [str(v_info['garage_id'])]
            l2 = [f"{v_info['start_product']}(0.0)"]
            c_p = v_info['start_product']

            for step in full_path:
                symbol = "[]" if step['node']['type'] == 'depot' else "()"
                l1.append(f"{step['node']['id']} {symbol[0]}{step['qty']}{symbol[1]}")
                
                step_cost = 0.0
                if step['prod'] != c_p:
                    step_cost = self.inst.transition_matrix[c_p][step['prod']]
                    c_p = step['prod']
                l2.append(f"{step['prod']}({step_cost:.1f})")
            
            l1.append(str(v_info['garage_id']))
            l2.append(f"{c_p}(0.0)")

            output.append(f"{v_info['id']}: " + " - ".join(l1))
            output.append(f"{v_info['id']}: " + " - ".join(l2))
            output.append("")
            g_total_dist += v_dist
            g_total_cost += v_cost

        # Pied de page
        output.append(f"{v_used_count}")
        output.append(f"{g_total_changes}")
        output.append(f"{g_total_cost:.2f}")
        output.append(f"{g_total_dist:.2f}")
        output.append("OR-Tools-SCIP")
        output.append("3.0")
        return "\n".join(output)

if __name__ == "__main__":
    file_path = "instance.dat"
    if os.path.exists(file_path):
        inst = MPVRPInstance(file_path)
        solver = MPVRPSolver(inst)
        res = solver.solve()
        if res:
            with open("Sol_" + os.path.basename(file_path), "w") as f: f.write(res)
            print("💾 Fichier solution généré.")
