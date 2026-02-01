"""
MPVRP Solver - Métaheuristique : Algorithme Génétique Hybride
Combine évolution génétique avec recherche locale pour optimiser la qualité
"""

import math
import random
import glob
from typing import List, Dict, Tuple


# ============================================================================
# CHARGEMENT DE L'INSTANCE
# ============================================================================

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
        self.dist_matrix = {}

        self._parse()
        self._compute_distances()

    def _parse(self):
        with open(self.filepath, 'r') as f:
            lines = [l.strip() for l in f.readlines() if l.strip() and not l.startswith('#')]

        dims = list(map(int, lines[0].split()))
        self.nb_products, self.nb_depots, self.nb_garages, self.nb_stations, self.nb_vehicles = dims

        current_line = 1

        # Matrice de transition
        for _ in range(self.nb_products):
            row = list(map(float, lines[current_line].split()))
            self.transition_matrix.append(row)
            current_line += 1

        # Véhicules
        for _ in range(self.nb_vehicles):
            parts = lines[current_line].split()
            v = {
                'id': int(parts[0]),
                'capacity': float(parts[1]),
                'garage_id': int(parts[2]),
                'start_product': int(parts[3]) - 1
            }
            self.vehicles.append(v)
            current_line += 1

        # Dépôts
        for _ in range(self.nb_depots):
            parts = lines[current_line].split()
            stocks = list(map(float, parts[3:]))
            d = {
                'type': 'depot',
                'id': int(parts[0]),
                'x': float(parts[1]),
                'y': float(parts[2]),
                'stocks': stocks.copy(),
                'original_stocks': stocks.copy()
            }
            self.depots.append(d)
            self.nodes.append(d)
            current_line += 1

        # Garages
        for _ in range(self.nb_garages):
            parts = lines[current_line].split()
            g = {
                'type': 'garage',
                'id': int(parts[0]),
                'x': float(parts[1]),
                'y': float(parts[2])
            }
            self.garages.append(g)
            self.nodes.append(g)
            current_line += 1

        # Stations
        for _ in range(self.nb_stations):
            parts = lines[current_line].split()
            demands = list(map(float, parts[3:]))
            s = {
                'type': 'station',
                'id': int(parts[0]),
                'x': float(parts[1]),
                'y': float(parts[2]),
                'demands': demands
            }
            self.stations.append(s)
            self.nodes.append(s)
            current_line += 1

    def _compute_distances(self):
        for i, n1 in enumerate(self.nodes):
            for j, n2 in enumerate(self.nodes):
                dist = math.sqrt((n1['x'] - n2['x'])**2 + (n1['y'] - n2['y'])**2)
                self.dist_matrix[(i, j)] = dist


# ============================================================================
# STRUCTURE DE SOLUTION
# ============================================================================

class MiniRoute:
    """Une mini-route = un produit + un dépôt + des livraisons"""
    def __init__(self, product, depot_id=None):
        self.product = product
        self.depot_id = depot_id
        self.deliveries = []  # Liste de (station_id, quantity)

    def add_delivery(self, station_id, quantity):
        self.deliveries.append((station_id, quantity))

    def get_total_load(self):
        return sum(qty for _, qty in self.deliveries)

    def copy(self):
        new = MiniRoute(self.product, self.depot_id)
        new.deliveries = self.deliveries.copy()
        return new


class Solution:
    """Solution complète : routes par véhicule"""
    def __init__(self, instance):
        self.instance = instance
        self.vehicle_routes = {v['id']: [] for v in instance.vehicles}
        self.fitness = float('inf')
        self.total_distance = 0.0
        self.total_cleaning_cost = 0.0
        self.nb_cleanings = 0

    def copy(self):
        new = Solution(self.instance)
        new.vehicle_routes = {
            vid: [route.copy() for route in routes]
            for vid, routes in self.vehicle_routes.items()
        }
        new.fitness = self.fitness
        new.total_distance = self.total_distance
        new.total_cleaning_cost = self.total_cleaning_cost
        new.nb_cleanings = self.nb_cleanings
        return new

    def evaluate(self):
        """Calcule la fitness de la solution"""
        self.total_distance = 0.0
        self.total_cleaning_cost = 0.0
        self.nb_cleanings = 0
        penalty = 0.0

        # Vérifier satisfaction des demandes
        demand_delivered = {}
        for station in self.instance.stations:
            for p in range(self.instance.nb_products):
                demand_delivered[(station['id'], p)] = 0.0

        # Vérifier utilisation des stocks
        stock_used = {}
        for depot in self.instance.depots:
            for p in range(self.instance.nb_products):
                stock_used[(depot['id'], p)] = 0.0

        # Calculer coûts et livraisons
        for vehicle_id, routes in self.vehicle_routes.items():
            vehicle = next(v for v in self.instance.vehicles if v['id'] == vehicle_id)
            garage = next(g for g in self.instance.garages if g['id'] == vehicle['garage_id'])

            current_product = vehicle['start_product']
            prev_node = garage

            for route in routes:
                # Vérifier capacité
                load = route.get_total_load()
                if load > vehicle['capacity']:
                    penalty += (load - vehicle['capacity']) * 1000

                # Comptabiliser utilisation du stock
                if route.depot_id is not None:
                    stock_used[(route.depot_id, route.product)] += load

                # Coût de nettoyage
                if route.product != current_product:
                    cost = self.instance.transition_matrix[current_product][route.product]
                    self.total_cleaning_cost += cost
                    self.nb_cleanings += 1
                    current_product = route.product

                # Distance vers dépôt
                if route.depot_id is not None:
                    depot = next(d for d in self.instance.depots if d['id'] == route.depot_id)
                    self.total_distance += self._calc_dist(prev_node, depot)
                    prev_node = depot

                # Distance vers stations et comptabiliser livraisons
                for station_id, qty in route.deliveries:
                    station = next(s for s in self.instance.stations if s['id'] == station_id)
                    self.total_distance += self._calc_dist(prev_node, station)
                    prev_node = station

                    # Comptabiliser la livraison
                    demand_delivered[(station_id, route.product)] += qty

            # Retour garage
            if routes:
                self.total_distance += self._calc_dist(prev_node, garage)

        # Pénalité pour demandes non satisfaites
        for station in self.instance.stations:
            for p in range(self.instance.nb_products):
                expected = station['demands'][p]
                delivered = demand_delivered[(station['id'], p)]
                if abs(delivered - expected) > 0.01:
                    penalty += abs(delivered - expected) * 100

        # Pénalité pour stocks dépassés
        for depot in self.instance.depots:
            for p in range(self.instance.nb_products):
                available = depot['original_stocks'][p]
                used = stock_used[(depot['id'], p)]
                if used > available + 0.01:
                    penalty += (used - available) * 200  # Pénalité forte

        self.fitness = self.total_distance + self.total_cleaning_cost + penalty
        return self.fitness

    def _calc_dist(self, n1, n2):
        return math.sqrt((n1['x'] - n2['x'])**2 + (n1['y'] - n2['y'])**2)


# ============================================================================
# HEURISTIQUE CONSTRUCTIVE (SOLUTION INITIALE)
# ============================================================================

def create_greedy_solution(instance):
    """Crée une solution gloutonne respectant les contraintes de stock"""
    solution = Solution(instance)

    # Copier les stocks pour ne pas modifier l'instance originale
    available_stocks = {}
    for depot in instance.depots:
        available_stocks[depot['id']] = depot['stocks'].copy()

    # Créer toutes les demandes à satisfaire
    remaining_demands = {}
    for station in instance.stations:
        for p in range(instance.nb_products):
            if station['demands'][p] > 0:
                key = (station['id'], p)
                remaining_demands[key] = {
                    'station_id': station['id'],
                    'station': station,
                    'product': p,
                    'quantity': station['demands'][p]
                }

    # Tant qu'il reste des demandes à satisfaire
    iteration = 0
    max_iterations = len(remaining_demands) * 10

    while remaining_demands and iteration < max_iterations:
        iteration += 1
        improved = False

        # Trier les demandes par quantité (grandes d'abord)
        demands_list = sorted(remaining_demands.values(),
                             key=lambda d: d['quantity'], reverse=True)

        for demand in demands_list:
            key = (demand['station_id'], demand['product'])

            # Trouver le meilleur (véhicule, dépôt) pour cette demande
            best_assignment = None
            best_cost = float('inf')

            for vehicle in instance.vehicles:
                garage = next(g for g in instance.garages if g['id'] == vehicle['garage_id'])

                for depot in instance.depots:
                    stock_available = available_stocks[depot['id']][demand['product']]

                    if stock_available <= 0:
                        continue

                    # Quantité qu'on peut livrer (limitée par capacité et stock)
                    deliverable = min(demand['quantity'],
                                    vehicle['capacity'],
                                    stock_available)

                    if deliverable <= 0:
                        continue

                    # Coût (distance)
                    dist = math.sqrt((garage['x'] - depot['x'])**2 +
                                   (garage['y'] - depot['y'])**2)
                    dist += math.sqrt((depot['x'] - demand['station']['x'])**2 +
                                    (depot['y'] - demand['station']['y'])**2)

                    # Préférer les livraisons complètes
                    if deliverable == demand['quantity']:
                        dist *= 0.8  # Bonus pour livraison complète

                    if dist < best_cost:
                        best_cost = dist
                        best_assignment = {
                            'vehicle': vehicle,
                            'depot': depot,
                            'quantity': deliverable
                        }

            # Si on a trouvé une assignation
            if best_assignment:
                vehicle = best_assignment['vehicle']
                depot = best_assignment['depot']
                qty = best_assignment['quantity']

                # Chercher si on peut fusionner avec une route existante
                merged = False
                for route in solution.vehicle_routes[vehicle['id']]:
                    if (route.product == demand['product'] and
                        route.depot_id == depot['id'] and
                        route.get_total_load() + qty <= vehicle['capacity']):
                        # Fusionner avec cette route
                        route.add_delivery(demand['station_id'], qty)
                        merged = True
                        break

                if not merged:
                    # Créer une nouvelle route
                    route = MiniRoute(demand['product'], depot['id'])
                    route.add_delivery(demand['station_id'], qty)
                    solution.vehicle_routes[vehicle['id']].append(route)

                # Mettre à jour les stocks et demandes
                available_stocks[depot['id']][demand['product']] -= qty
                demand['quantity'] -= qty

                # Si demande complètement satisfaite, la retirer
                if demand['quantity'] <= 0.01:
                    del remaining_demands[key]

                improved = True
                break  # Passer à la prochaine itération

        # Si aucune amélioration, arrêter pour éviter boucle infinie
        if not improved:
            break

    solution.evaluate()
    return solution


# ============================================================================
# ALGORITHME GÉNÉTIQUE HYBRIDE
# ============================================================================

class GeneticAlgorithm:
    def __init__(self, instance, population_size=30, generations=100):
        self.instance = instance
        self.population_size = population_size
        self.generations = generations
        self.elite_size = max(3, population_size // 10)
        self.mutation_rate = 0.2

    def evolve(self):
        """Exécute l'algorithme génétique"""
        # Population initiale
        population = [create_greedy_solution(self.instance) for _ in range(self.population_size)]

        best_ever = min(population, key=lambda s: s.fitness)

        for gen in range(self.generations):
            # Sélection par tournoi
            parents = []
            for _ in range(self.population_size):
                tournament = random.sample(population, 3)
                parents.append(min(tournament, key=lambda s: s.fitness))

            # Croisement
            offspring = []
            for i in range(0, len(parents) - 1, 2):
                if random.random() < 0.8:
                    child = self._crossover(parents[i], parents[i+1])
                    self._repair_solution(child)  # Réparer après croisement
                else:
                    child = parents[i].copy()
                offspring.append(child)

            # Mutation
            for child in offspring:
                if random.random() < self.mutation_rate:
                    self._mutate(child)
                    self._repair_solution(child)  # Réparer après mutation

            # Recherche locale sur les meilleurs
            for child in offspring[:self.elite_size]:
                self._local_search(child, iterations=5)

            # Évaluation
            for child in offspring:
                child.evaluate()

            # Nouvelle génération (élitisme)
            population.sort(key=lambda s: s.fitness)
            population = population[:self.elite_size] + offspring[:self.population_size - self.elite_size]

            # Mise à jour du meilleur
            current_best = population[0]
            if current_best.fitness < best_ever.fitness:
                best_ever = current_best.copy()
                print(f"   Gen {gen}: Meilleure fitness = {best_ever.fitness:.2f}")

        return best_ever

    def _crossover(self, parent1, parent2):
        """Croisement uniforme"""
        child = Solution(self.instance)
        for vid in parent1.vehicle_routes.keys():
            if random.random() < 0.5:
                child.vehicle_routes[vid] = [r.copy() for r in parent1.vehicle_routes[vid]]
            else:
                child.vehicle_routes[vid] = [r.copy() for r in parent2.vehicle_routes[vid]]
        return child

    def _mutate(self, solution):
        """Mutation intelligente : plusieurs types d'opérateurs"""
        mutation_type = random.choice(['swap_routes', 'split_route', 'merge_routes', 'change_depot'])

        if mutation_type == 'swap_routes':
            # Échanger deux routes entre véhicules
            vehicles = [v for v, routes in solution.vehicle_routes.items() if routes]
            if len(vehicles) >= 2:
                v1, v2 = random.sample(vehicles, 2)
                if solution.vehicle_routes[v1] and solution.vehicle_routes[v2]:
                    idx1 = random.randint(0, len(solution.vehicle_routes[v1]) - 1)
                    idx2 = random.randint(0, len(solution.vehicle_routes[v2]) - 1)
                    solution.vehicle_routes[v1][idx1], solution.vehicle_routes[v2][idx2] = \
                        solution.vehicle_routes[v2][idx2], solution.vehicle_routes[v1][idx1]

        elif mutation_type == 'split_route':
            # Diviser une route en deux si elle a plusieurs livraisons
            for vid, routes in solution.vehicle_routes.items():
                for route in routes:
                    if len(route.deliveries) >= 2:
                        # Diviser en deux routes
                        mid = len(route.deliveries) // 2
                        route1 = MiniRoute(route.product, route.depot_id)
                        route2 = MiniRoute(route.product, route.depot_id)
                        route1.deliveries = route.deliveries[:mid]
                        route2.deliveries = route.deliveries[mid:]
                        solution.vehicle_routes[vid].remove(route)
                        solution.vehicle_routes[vid].extend([route1, route2])
                        return

        elif mutation_type == 'merge_routes':
            # Fusionner deux routes du même produit
            for vid, routes in solution.vehicle_routes.items():
                if len(routes) >= 2:
                    for i in range(len(routes)):
                        for j in range(i + 1, len(routes)):
                            if routes[i].product == routes[j].product and routes[i].depot_id == routes[j].depot_id:
                                # Fusionner
                                routes[i].deliveries.extend(routes[j].deliveries)
                                routes.pop(j)
                                return

        elif mutation_type == 'change_depot':
            # Changer le dépôt d'une route
            for vid, routes in solution.vehicle_routes.items():
                if routes:
                    route = random.choice(routes)
                    new_depot = random.choice(self.instance.depots)
                    route.depot_id = new_depot['id']
                    return

    def _repair_solution(self, solution):
        """Répare une solution pour respecter les contraintes de stock"""
        # Calculer l'utilisation actuelle des stocks
        stock_used = {}
        for depot in self.instance.depots:
            for p in range(self.instance.nb_products):
                stock_used[(depot['id'], p)] = 0.0

        for routes in solution.vehicle_routes.values():
            for route in routes:
                if route.depot_id is not None:
                    stock_used[(route.depot_id, route.product)] += route.get_total_load()

        # Vérifier et corriger les dépassements
        for depot in self.instance.depots:
            for p in range(self.instance.nb_products):
                available = depot['original_stocks'][p]
                used = stock_used[(depot['id'], p)]

                if used > available + 0.01:
                    # Dépassement : redistribuer vers d'autres dépôts
                    excess = used - available

                    # Trouver les routes utilisant ce dépôt pour ce produit
                    affected_routes = []
                    for vid, routes in solution.vehicle_routes.items():
                        for route in routes:
                            if route.depot_id == depot['id'] and route.product == p:
                                affected_routes.append((vid, route))

                    # Redistribuer l'excès vers d'autres dépôts
                    for vid, route in affected_routes:
                        if excess <= 0:
                            break

                        # Chercher un autre dépôt avec stock disponible
                        for other_depot in self.instance.depots:
                            if other_depot['id'] == depot['id']:
                                continue

                            other_available = other_depot['original_stocks'][p]
                            other_used = stock_used[(other_depot['id'], p)]

                            if other_used < other_available:
                                # Changer le dépôt de cette route
                                route.depot_id = other_depot['id']
                                route_load = route.get_total_load()
                                stock_used[(depot['id'], p)] -= route_load
                                stock_used[(other_depot['id'], p)] += route_load
                                excess -= route_load
                                break

    def _local_search(self, solution, iterations=10):
        """Recherche locale : amélioration par échanges + réparation"""
        for _ in range(iterations):
            improved = False

            for vid, routes in solution.vehicle_routes.items():
                if len(routes) < 2:
                    continue

                # Essayer de fusionner deux routes du même produit
                for i in range(len(routes)):
                    for j in range(i + 1, len(routes)):
                        if routes[i].product == routes[j].product:
                            # Vérifier si la fusion est possible
                            vehicle = next(v for v in self.instance.vehicles if v['id'] == vid)
                            total_load = routes[i].get_total_load() + routes[j].get_total_load()

                            if total_load <= vehicle['capacity']:
                                # Fusionner
                                merged = routes[i].copy()
                                merged.deliveries.extend(routes[j].deliveries)

                                old_fitness = solution.fitness
                                temp_routes = routes[:i] + routes[i+1:j] + routes[j+1:] + [merged]
                                solution.vehicle_routes[vid] = temp_routes

                                # Réparer si nécessaire
                                self._repair_solution(solution)
                                solution.evaluate()

                                if solution.fitness >= old_fitness:
                                    # Annuler
                                    solution.vehicle_routes[vid] = routes
                                    solution.fitness = old_fitness
                                else:
                                    improved = True
                                    break
                    if improved:
                        break


# ============================================================================
# SOLVEUR PRINCIPAL
# ============================================================================

class MPVRPSolver:
    def __init__(self, instance):
        self.inst = instance

    def solve(self):
        """
        Résout le problème avec l'algorithme génétique hybride
        """
        print("🧬 Résolution par Algorithme Génétique Hybride...")

        # Paramètres adaptés à la taille
        nb_stations = self.inst.nb_stations
        if nb_stations <= 10:
            pop_size, gens = 20, 50
        elif nb_stations <= 30:
            pop_size, gens = 30, 100
        else:
            pop_size, gens = 40, 150

        print(f"   Population: {pop_size}, Générations: {gens}")

        # Algorithme génétique
        ga = GeneticAlgorithm(self.inst, population_size=pop_size, generations=gens)
        best_solution = ga.evolve()

        print(f"✅ Solution trouvée !")
        print(f"   Fitness: {best_solution.fitness:.2f}")
        print(f"   Distance: {best_solution.total_distance:.2f}")
        print(f"   Nettoyages: {best_solution.nb_cleanings} (coût: {best_solution.total_cleaning_cost:.2f})")

        return self._format_solution(best_solution)

    def _format_solution(self, solution):
        """Formate la solution au format texte requis"""
        output_lines = []
        grand_total_dist = 0.0
        grand_total_cost = 0.0
        grand_total_changes = 0
        nb_vehicles_used = 0

        def calc_dist(n1, n2):
            return math.sqrt((n1['x'] - n2['x'])**2 + (n1['y'] - n2['y'])**2)

        for vehicle in self.inst.vehicles:
            v_id = vehicle['id']
            routes = solution.vehicle_routes.get(v_id, [])

            if not routes:
                continue

            nb_vehicles_used += 1
            garage = next(g for g in self.inst.garages if g['id'] == vehicle['garage_id'])

            line1_parts = [f"{vehicle['garage_id']}"]
            line2_parts = [f"{vehicle['start_product']}(0.0)"]

            current_product = vehicle['start_product']
            previous_node = garage

            for route in routes:
                prod = route.product

                # Coût de transition
                transition_cost = 0.0
                if prod != current_product:
                    transition_cost = self.inst.transition_matrix[current_product][prod]
                    grand_total_cost += transition_cost
                    grand_total_changes += 1
                    current_product = prod

                # Dépôt
                if route.depot_id is not None:
                    depot = next(d for d in self.inst.depots if d['id'] == route.depot_id)
                    qty_int = int(round(route.get_total_load()))

                    line1_parts.append(f"{depot['id']} [{qty_int}]")
                    line2_parts.append(f"{prod}({transition_cost:.1f})")

                    grand_total_dist += calc_dist(previous_node, depot)
                    previous_node = depot
                    transition_cost = 0.0

                # Stations
                for station_id, qty in route.deliveries:
                    station = next(s for s in self.inst.stations if s['id'] == station_id)
                    qty_int = int(round(qty))

                    line1_parts.append(f"{station['id']} ({qty_int})")
                    line2_parts.append(f"{prod}(0.0)")

                    grand_total_dist += calc_dist(previous_node, station)
                    previous_node = station

            # Retour garage
            line1_parts.append(f"{vehicle['garage_id']}")
            line2_parts.append(f"{current_product}(0.0)")
            grand_total_dist += calc_dist(previous_node, garage)

            output_lines.append(f"{vehicle['id']}: " + " - ".join(line1_parts))
            output_lines.append(f"{vehicle['id']}: " + " - ".join(line2_parts))
            output_lines.append("")

        output_lines.append(f"{nb_vehicles_used}")
        output_lines.append(f"{grand_total_changes}")
        output_lines.append(f"{grand_total_cost:.2f}")
        output_lines.append(f"{grand_total_dist:.2f}")
        output_lines.append("Genetic-Hybrid")
        output_lines.append("1.0")

        return "\n".join(output_lines)


# ============================================================================
# POINT D'ENTRÉE PRINCIPAL
# ============================================================================

if __name__ == "__main__":
    # Récupère tous les fichiers .dat commençant par MPVRP_S_
    fichiers_a_traiter = sorted(glob.glob("MPVRP_L_*.dat"))

    if not fichiers_a_traiter:
        print("Aucun fichier .dat détecté dans le répertoire.")
    else:
        print(f"{len(fichiers_a_traiter)} fichiers à traiter.")

    for file_path in fichiers_a_traiter:
        # Éviter de traiter les fichiers solutions si on relance le script
        if file_path.startswith("Sol_"):
            continue

        print(f"\n{'='*70}")
        print(f"En cours : {file_path}")
        print(f"{'='*70}")

        try:
            # Chargement de l'instance
            inst = MPVRPInstance(file_path)
            print(f"📊 Instance: {inst.nb_stations} stations, {inst.nb_vehicles} véhicules, "
                  f"{inst.nb_products} produits")

            # Initialisation du solveur
            solver = MPVRPSolver(inst)

            # Lancement de la résolution
            res = solver.solve()

            if res:
                # Sauvegarde au format .dat
                out_name = "Sol_" + file_path
                with open(out_name, "w") as f:
                    f.write(res)
                print(f"✅ Fichier sauvegardé : {out_name}")
            else:
                print(f"❌ Échec de résolution pour {file_path}")

        except Exception as e:
            print(f"❌ Erreur lors du traitement de {file_path} : {e}")
            import traceback
            traceback.print_exc()

    print(f"\n{'='*70}")
    print("✅ Traitement terminé.")
    print(f"{'='*70}")
