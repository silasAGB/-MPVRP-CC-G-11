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
        if not self.solver: 
            self.solver = pywraplp.Solver.CreateSolver('CBC')

        self.MAX_MINI_ROUTES = 10
        self.solver.set_time_limit(120000)
        self.infinity = self.solver.infinity()

    def solve(self):
        print("🛠️ Construction du modèle (COMPLET ET CORRIGÉ)...")

        x = {}      # Routage: x[k,r,i,j] = 1 si véhicule k va de i à j sur route r
        y = {}      # Produit: y[k,r,p] = 1 si véhicule k transporte produit p sur route r
        q = {}      # Livraison: q[k,r,i,p] = quantité livrée à station i
        load = {}   # Chargement: load[k,r,d,p] = quantité chargée au dépôt d
        delta = {}  # Nettoyage: delta[k,r] = 1 si nettoyage requis

        all_indices = range(len(self.inst.nodes))
        depot_indices = [i for i, n in enumerate(self.inst.nodes) if n['type'] == 'depot']
        station_indices = [i for i, n in enumerate(self.inst.nodes) if n['type'] == 'station']
        garage_indices = [i for i, n in enumerate(self.inst.nodes) if n['type'] == 'garage']

        # ========== VARIABLES ==========
        for k in range(self.inst.nb_vehicles):
            cap = self.inst.vehicles[k]['capacity']
            
            for r in range(self.MAX_MINI_ROUTES):
                delta[k, r] = self.solver.BoolVar(f'd_{k}_{r}')

                for p in range(self.inst.nb_products):
                    y[k, r, p] = self.solver.BoolVar(f'y_{k}_{r}_{p}')

                    # Variables de chargement par dépôt
                    for d_idx in depot_indices:
                        load[k, r, d_idx, p] = self.solver.NumVar(0, cap, f'load_{k}_{r}_{d_idx}_{p}')

                # Variables de routage
                for i in all_indices:
                    for j in all_indices:
                        if i != j:
                            x[k, r, i, j] = self.solver.BoolVar(f'x_{k}_{r}_{i}_{j}')

                    # Variables de livraison
                    if self.inst.nodes[i]['type'] == 'station':
                        for p in range(self.inst.nb_products):
                            if self.inst.nodes[i]['demands'][p] > 0:
                                q[k, r, i, p] = self.solver.NumVar(0, cap, f'q_{k}_{r}_{i}_{p}')

        print("🔗 Ajout des contraintes...")

        # ========== CONTRAINTE 1 : SATISFACTION DEMANDE ==========
        print("   [1/10] Satisfaction demande...")
        for i in station_indices:
            node = self.inst.nodes[i]
            for p in range(self.inst.nb_products):
                if node['demands'][p] > 0:
                    self.solver.Add(
                        sum(q.get((k, r, i, p), 0) 
                            for k in range(self.inst.nb_vehicles) 
                            for r in range(self.MAX_MINI_ROUTES))
                        == node['demands'][p],
                        f"demand_station_{i}_prod_{p}"
                    )

        # ========== CONTRAINTE 2 : STOCK DÉPÔTS ==========
        print("   [2/10] Stocks dépôts...")
        for d_idx in depot_indices:
            # Trouver l'objet dépôt correspondant
            depot_obj = next(d for d in self.inst.depots 
                           if d['id'] == self.inst.nodes[d_idx]['id'])
            
            for p in range(self.inst.nb_products):
                self.solver.Add(
                    sum(load.get((k, r, d_idx, p), 0)
                        for k in range(self.inst.nb_vehicles)
                        for r in range(self.MAX_MINI_ROUTES))
                    <= depot_obj['stocks'][p],
                    f"stock_depot_{d_idx}_prod_{p}"
                )

        # ========== CONTRAINTE 3 : CAPACITÉ VÉHICULE PAR MINI-ROUTE ==========
        print("   [3/10] Capacité véhicules...")
        for k in range(self.inst.nb_vehicles):
            cap = self.inst.vehicles[k]['capacity']
            
            for r in range(self.MAX_MINI_ROUTES):
                # ✅ CRITIQUE : Total chargé sur UNE mini-route <= capacité
                total_loaded_on_route = sum(
                    load.get((k, r, d_idx, p), 0)
                    for d_idx in depot_indices
                    for p in range(self.inst.nb_products)
                )
                self.solver.Add(
                    total_loaded_on_route <= cap,
                    f"capacity_vehicle_{k}_route_{r}"
                )

        # ========== CONTRAINTE 4 : MONO-PRODUIT PAR MINI-ROUTE ==========
        print("   [4/10] Mono-produit...")
        for k in range(self.inst.nb_vehicles):
            for r in range(self.MAX_MINI_ROUTES):
                # Un seul produit par mini-route
                self.solver.Add(
                    sum(y[k, r, p] for p in range(self.inst.nb_products)) <= 1,
                    f"mono_product_v{k}_r{r}"
                )

        # ========== CONTRAINTE 5 : CONSERVATION MASSE PAR MINI-ROUTE ==========
        print("   [5/11] Conservation masse...")
        for k in range(self.inst.nb_vehicles):
            cap = self.inst.vehicles[k]['capacity']
            
            for r in range(self.MAX_MINI_ROUTES):
                for p in range(self.inst.nb_products):
                    total_loaded = sum(
                        load.get((k, r, d_idx, p), 0) 
                        for d_idx in depot_indices
                    )
                    
                    total_delivered = sum(
                        q.get((k, r, i, p), 0)
                        for i in station_indices
                    )
                    
                    self.solver.Add(
                        total_loaded == total_delivered,
                        f"mass_conservation_v{k}_r{r}_p{p}"
                    )
                    
                    self.solver.Add(
                        total_delivered <= cap * y[k, r, p],
                        f"delivery_requires_product_v{k}_r{r}_p{p}"
                    )
                    
                    self.solver.Add(
                        total_loaded <= cap * y[k, r, p],
                        f"loading_requires_product_v{k}_r{r}_p{p}"
                    )

        # ========== ✅ NOUVELLE CONTRAINTE 5bis : UN SEUL DÉPÔT PAR MINI-ROUTE ==========
        print("   [5bis/11] Un seul dépôt par mini-route...")
        for k in range(self.inst.nb_vehicles):
            cap = self.inst.vehicles[k]['capacity']
            
            for r in range(self.MAX_MINI_ROUTES):
                # Nombre de dépôts visités sur cette mini-route
                depot_visits = []
                for d_idx in depot_indices:
                    # Variable binaire : ce dépôt est-il visité ?
                    depot_visited = self.solver.BoolVar(f'depot_visit_{k}_{r}_{d_idx}')
                    depot_visits.append(depot_visited)
                    
                    # Lien avec le flux sortant du dépôt
                    outgoing = sum(
                        x.get((k, r, d_idx, j), 0)
                        for j in all_indices if j != d_idx
                    )
                    
                    # Si on sort du dépôt, alors depot_visited = 1
                    self.solver.Add(depot_visited >= outgoing)
                    self.solver.Add(depot_visited <= outgoing * len(all_indices))
                    
                    # Si on ne visite pas le dépôt, on ne peut rien y charger
                    for p in range(self.inst.nb_products):
                        self.solver.Add(
                            load.get((k, r, d_idx, p), 0) <= cap * depot_visited,
                            f"load_requires_depot_visit_v{k}_r{r}_d{d_idx}_p{p}"
                        )
                
                # ✅ CONTRAINTE CLÉE : Maximum 1 dépôt par mini-route
                self.solver.Add(
                    sum(depot_visits) <= 1,
                    f"max_one_depot_v{k}_r{r}"
                )

        # ========== CONTRAINTE 6 : LIEN CHARGEMENT <-> VISITE DÉPÔT ==========
        print("   [6/10] Lien chargement-visite dépôt...")
        for k in range(self.inst.nb_vehicles):
            cap = self.inst.vehicles[k]['capacity']
            
            for r in range(self.MAX_MINI_ROUTES):
                for d_idx in depot_indices:
                    # Flux sortant du dépôt
                    outgoing = sum(
                        x.get((k, r, d_idx, j), 0)
                        for j in all_indices if j != d_idx
                    )
                    
                    for p in range(self.inst.nb_products):
                        # ✅ On ne peut charger que si on visite le dépôt
                        self.solver.Add(
                            load.get((k, r, d_idx, p), 0) <= cap * outgoing,
                            f"load_requires_visit_v{k}_r{r}_d{d_idx}_p{p}"
                        )

        # ========== CONTRAINTE 7 : LIEN LIVRAISON <-> VISITE STATION ==========
        print("   [7/10] Lien livraison-visite station...")
        for k in range(self.inst.nb_vehicles):
            cap = self.inst.vehicles[k]['capacity']
            
            for r in range(self.MAX_MINI_ROUTES):
                for i in station_indices:
                    # Flux entrant à la station
                    incoming = sum(
                        x.get((k, r, j, i), 0)
                        for j in all_indices if j != i
                    )
                    
                    for p in range(self.inst.nb_products):
                        if (k, r, i, p) in q:
                            # ✅ On ne peut livrer que si on visite la station
                            self.solver.Add(
                                q[k, r, i, p] <= cap * incoming,
                                f"delivery_requires_visit_v{k}_r{r}_s{i}_p{p}"
                            )

        # ========== CONTRAINTE 8 : CONSERVATION FLUX ==========
        print("   [8/10] Conservation flux...")
        for k in range(self.inst.nb_vehicles):
            for r in range(self.MAX_MINI_ROUTES):
                for i in all_indices:
                    # Flux entrant
                    flow_in = sum(
                        x.get((k, r, j, i), 0)
                        for j in all_indices if j != i
                    )
                    
                    # Flux sortant
                    flow_out = sum(
                        x.get((k, r, i, j), 0)
                        for j in all_indices if j != i
                    )
                    
                    # ✅ Ce qui entre = ce qui sort (sauf garages)
                    if self.inst.nodes[i]['type'] != 'garage':
                        self.solver.Add(
                            flow_in == flow_out,
                            f"flow_conservation_v{k}_r{r}_node{i}"
                        )

        # ========== CONTRAINTE 9 : DÉPART/RETOUR GARAGE ==========
        print("   [9/10] Garage départ/retour...")
        for k in range(self.inst.nb_vehicles):
            garage_id = self.inst.vehicles[k]['garage_id']
            garage_idx = next(i for i, n in enumerate(self.inst.nodes) 
                             if n['type'] == 'garage' and n['id'] == garage_id)
            
            for r in range(self.MAX_MINI_ROUTES):
                # ✅ Si la route est utilisée, elle part et revient au garage
                route_used = sum(
                    x.get((k, r, i, j), 0)
                    for i in all_indices
                    for j in all_indices if i != j
                )
                
                # Départ du garage
                leaving_garage = sum(
                    x.get((k, r, garage_idx, j), 0)
                    for j in all_indices if j != garage_idx
                )
                
                # Retour au garage
                returning_garage = sum(
                    x.get((k, r, j, garage_idx), 0)
                    for j in all_indices if j != garage_idx
                )
                
                # Si route utilisée => 1 départ et 1 retour
                self.solver.Add(leaving_garage <= 1)
                self.solver.Add(returning_garage <= 1)
                
                # Symétrie : si on part, on revient
                self.solver.Add(leaving_garage == returning_garage)

        # ========== CONTRAINTE 10 : NETTOYAGE ==========
        print("   [10/10] Gestion nettoyage...")
        for k in range(self.inst.nb_vehicles):
            p_start = self.inst.vehicles[k]['start_product']
            
            # Première route : nettoyage si produit différent
            for p in range(self.inst.nb_products):
                if p != p_start:
                    self.solver.Add(delta[k, 0] >= y[k, 0, p])
            
            # Routes suivantes : nettoyage si changement
            for r in range(1, self.MAX_MINI_ROUTES):
                for p1 in range(self.inst.nb_products):
                    for p2 in range(self.inst.nb_products):
                        if p1 != p2:
                            self.solver.Add(
                                delta[k, r] >= y[k, r-1, p1] + y[k, r, p2] - 1
                            )

        # ========== FONCTION OBJECTIF ==========
        print("🎯 Définition objectif...")
        
        dist_obj = sum(
            self.inst.dist_matrix[i, j] * x.get((k, r, i, j), 0)
            for k in range(self.inst.nb_vehicles)
            for r in range(self.MAX_MINI_ROUTES)
            for i in all_indices
            for j in all_indices if i != j
        )
        
        clean_obj = sum(
            10.0 * delta[k, r]
            for k in range(self.inst.nb_vehicles)
            for r in range(self.MAX_MINI_ROUTES)
        )

        self.solver.Minimize(dist_obj + clean_obj)

        # ========== RÉSOLUTION ==========
        print("⏳ Résolution en cours...")
        status = self.solver.Solve()

        if status == pywraplp.Solver.OPTIMAL:
            print(f"✅ Solution OPTIMALE trouvée !")
            print(f"   Objectif: {self.solver.Objective().Value():.2f}")
            print(f"   Temps: {self.solver.WallTime():.2f}s")
            return self._extract_solution(x, y, q, load)
            
        elif status == pywraplp.Solver.FEASIBLE:
            print(f"⚠️  Solution FAISABLE trouvée")
            print(f"   Objectif: {self.solver.Objective().Value():.2f}")
            return self._extract_solution(x, y, q, load)
            
        else:
            print("❌ Pas de solution (Infeasible/Unbounded)")
            return None


    def _extract_solution(self, x, y, q, load):
      output_lines = []
      grand_total_dist = 0.0
      grand_total_cost = 0.0
      grand_total_changes = 0
      nb_vehicles_used = 0

      def get_node_by_id(node_id, node_type):
          if node_type == 'G':
              nodes = self.inst.garages
          elif node_type == 'D':
              nodes = self.inst.depots
          else:
              nodes = self.inst.stations
          for n in nodes:
              if n['id'] == node_id:
                  return n
          return None
      
      def calc_dist(n1, n2):
          return math.sqrt((n1['x'] - n2['x'])**2 + (n1['y'] - n2['y'])**2)
      
      def get_value(var):
          return var.solution_value() if var else 0.0

      depot_indices = [i for i, n in enumerate(self.inst.nodes) if n['type'] == 'depot']
      station_indices = [i for i, n in enumerate(self.inst.nodes) if n['type'] == 'station']

      for k in range(self.inst.nb_vehicles):
          vehicle = self.inst.vehicles[k]
          garage_node = get_node_by_id(vehicle['garage_id'], 'G')
          
          mini_routes = []
          
          for r in range(self.MAX_MINI_ROUTES):
              active_product = -1
              for p in range(self.inst.nb_products):
                  if get_value(y[k, r, p]) > 0.5:
                      active_product = p
                      break
              
              if active_product == -1:
                  continue
              
              depot_used = None
              depot_qty = 0
              for d_idx in depot_indices:
                  qty = get_value(load.get((k, r, d_idx, active_product), None))
                  if qty > 0.1:
                      depot_used = self.inst.nodes[d_idx]
                      depot_qty = qty
                      break
              
              stations_visited = []
              for s_idx in station_indices:
                  qty = get_value(q.get((k, r, s_idx, active_product), None))
                  if qty > 0.1:
                      stations_visited.append({
                          'node': self.inst.nodes[s_idx],
                          'qty': qty
                      })
              
              if depot_used or stations_visited:
                  mini_routes.append({
                      'product': active_product,
                      'depot': depot_used,
                      'depot_qty': depot_qty,
                      'stations': stations_visited
                  })
          
          # ✅ SAUT SI VÉHICULE NON UTILISÉ
          if len(mini_routes) == 0:
              continue  # ← NE PAS afficher ce véhicule
          
          nb_vehicles_used += 1
          
          # GÉNÉRATION LIGNES
          line1_parts = [f"{vehicle['garage_id']}"]
          line2_parts = [f"{vehicle['start_product']}(0.0)"]
          
          current_product = vehicle['start_product']
          previous_node = garage_node
          
          for mini_route in mini_routes:
              prod = mini_route['product']
              
              transition_cost = 0.0
              if prod != current_product:
                  transition_cost = self.inst.transition_matrix[current_product][prod]
                  grand_total_cost += transition_cost
                  grand_total_changes += 1
                  current_product = prod
              
              if mini_route['depot']:
                  depot = mini_route['depot']
                  qty_int = int(round(mini_route['depot_qty']))
                  
                  line1_parts.append(f"{depot['id']} [{qty_int}]")
                  line2_parts.append(f"{prod}({transition_cost:.1f})")
                  
                  grand_total_dist += calc_dist(previous_node, depot)
                  previous_node = depot
                  transition_cost = 0.0
              
              for station_info in mini_route['stations']:
                  station = station_info['node']
                  qty_int = int(round(station_info['qty']))
                  
                  line1_parts.append(f"{station['id']} ({qty_int})")
                  line2_parts.append(f"{prod}(0.0)")
                  
                  grand_total_dist += calc_dist(previous_node, station)
                  previous_node = station
          
          line1_parts.append(f"{vehicle['garage_id']}")
          line2_parts.append(f"{current_product}(0.0)")
          grand_total_dist += calc_dist(previous_node, garage_node)
          
          # ✅ AFFICHER SEULEMENT LES VÉHICULES UTILISÉS
          output_lines.append(f"{vehicle['id']}: " + " - ".join(line1_parts))
          output_lines.append(f"{vehicle['id']}: " + " - ".join(line2_parts))
          output_lines.append("")
      
      output_lines.append(f"{nb_vehicles_used}")
      output_lines.append(f"{grand_total_changes}")
      output_lines.append(f"{grand_total_cost:.2f}")
      output_lines.append(f"{grand_total_dist:.2f}")
      output_lines.append("OR-Tools-SCIP")
      output_lines.append("3.2")
      
      return "\n".join(output_lines)

import glob

if __name__ == "__main__":
    # Récupère tous les fichiers .dat commençant par MPVRP_S_
    fichiers_a_traiter = sorted(glob.glob("MPVRP_S_*.dat"))

    if not fichiers_a_traiter:
        print("Aucun fichier .dat détecté dans le répertoire.")
    else:
        print(f"{len(fichiers_a_traiter)} fichiers à traiter.")

    for file_path in fichiers_a_traiter:
        # Éviter de traiter les fichiers solutions si on relance le script
        if file_path.startswith("Sol_"):
            continue

        print(f"En cours : {file_path}")

        try:
            # Chargement de l'instance
            inst = MPVRPInstance(file_path)

            # Initialisation du solveur
            solver = MPVRPSolver(inst)

            # Lancement de la résolution
            res = solver.solve()

            if res:
                # Sauvegarde au format .dat
                out_name = "Sol_" + file_path
                with open(out_name, "w") as f:
                    f.write(res)
                print(f"Fichier sauvegardé : {out_name}")
            else:
                print(f"Échec de résolution pour {file_path}")

        except Exception as e:
            print(f"Erreur lors du traitement de {file_path} : {e}")

    print("Traitement terminé.")
