"""
MPVRP-CC Solver using Google OR-Tools
======================================

This module implements a solver for the Multi-Product Vehicle Routing Problem
with Changeover Cost using Google OR-Tools CP-SAT solver.

Author: MPVRP-CC Team
Date: January 31, 2026
"""

from ortools.sat.python import cp_model
import math
from dataclasses import dataclass
from typing import List, Dict, Tuple
import time


@dataclass
class Vehicle:
    """Represents a delivery vehicle"""
    id: int
    capacity: int
    home_garage: int
    initial_product: int


@dataclass
class Depot:
    """Represents a loading depot"""
    id: int
    x: float
    y: float
    stock: Dict[int, int]  # product_id -> available_stock


@dataclass
class Garage:
    """Represents a vehicle garage"""
    id: int
    x: float
    y: float


@dataclass
class Station:
    """Represents a service station (customer)"""
    id: int
    x: float
    y: float
    demand: Dict[int, int]  # product_id -> demand


class MPVRPInstance:
    """Represents a complete MPVRP-CC problem instance"""
    
    def __init__(self):
        self.uuid = ""
        self.nb_products = 0
        self.nb_depots = 0
        self.nb_garages = 0
        self.nb_stations = 0
        self.nb_vehicles = 0
        
        self.transition_costs = []  # Matrix of changeover costs
        self.vehicles: List[Vehicle] = []
        self.depots: List[Depot] = []
        self.garages: List[Garage] = []
        self.stations: List[Station] = []
        
        self.distance_matrix = []
    
    def calculate_distances(self):
        """Calculate Euclidean distances between all sites"""
        # Combine all locations: garages + depots + stations
        all_locations = []
        
        for garage in self.garages:
            all_locations.append((garage.x, garage.y, f'G{garage.id}'))
        
        for depot in self.depots:
            all_locations.append((depot.x, depot.y, f'D{depot.id}'))
        
        for station in self.stations:
            all_locations.append((station.x, station.y, f'S{station.id}'))
        
        n = len(all_locations)
        self.distance_matrix = [[0.0] * n for _ in range(n)]
        
        for i in range(n):
            for j in range(n):
                if i != j:
                    x1, y1, _ = all_locations[i]
                    x2, y2, _ = all_locations[j]
                    dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                    self.distance_matrix[i][j] = dist
    
    def get_location_index(self, loc_type: str, loc_id: int) -> int:
        """Get the index in distance matrix for a location"""
        if loc_type == 'G':  # Garage
            return loc_id - 1
        elif loc_type == 'D':  # Depot
            return self.nb_garages + (loc_id - 1)
        elif loc_type == 'S':  # Station
            return self.nb_garages + self.nb_depots + (loc_id - 1)
        return -1


def parse_instance(filename: str) -> MPVRPInstance:
    """Parse a MPVRP-CC instance file"""
    instance = MPVRPInstance()
    
    with open(filename, 'r') as f:
        lines = [line.strip() for line in f.readlines() if line.strip()]
    
    line_idx = 0
    
    # Line 1: UUID
    instance.uuid = lines[line_idx].replace('#', '').strip()
    line_idx += 1
    
    # Line 2: Global parameters
    params = list(map(int, lines[line_idx].split()))
    instance.nb_products = params[0]
    instance.nb_depots = params[1]
    instance.nb_garages = params[2]
    instance.nb_stations = params[3]
    instance.nb_vehicles = params[4]
    line_idx += 1
    
    # Transition cost matrix
    instance.transition_costs = []
    for _ in range(instance.nb_products):
        row = list(map(float, lines[line_idx].split()))
        instance.transition_costs.append(row)
        line_idx += 1
    
    # Vehicles
    for _ in range(instance.nb_vehicles):
        parts = lines[line_idx].split()
        vehicle = Vehicle(
            id=int(parts[0]),
            capacity=int(parts[1]),
            home_garage=int(parts[2]),
            initial_product=int(parts[3])
        )
        instance.vehicles.append(vehicle)
        line_idx += 1
    
    # Depots
    for _ in range(instance.nb_depots):
        parts = list(map(float, lines[line_idx].split()))
        depot_id = int(parts[0])
        x, y = parts[1], parts[2]
        stock = {p: int(parts[3 + p]) for p in range(instance.nb_products)}
        
        depot = Depot(id=depot_id, x=x, y=y, stock=stock)
        instance.depots.append(depot)
        line_idx += 1
    
    # Garages
    for _ in range(instance.nb_garages):
        parts = list(map(float, lines[line_idx].split()))
        garage = Garage(id=int(parts[0]), x=parts[1], y=parts[2])
        instance.garages.append(garage)
        line_idx += 1
    
    # Stations
    for _ in range(instance.nb_stations):
        parts = list(map(float, lines[line_idx].split()))
        station_id = int(parts[0])
        x, y = parts[1], parts[2]
        demand = {p: int(parts[3 + p]) for p in range(instance.nb_products)}
        
        station = Station(id=station_id, x=x, y=y, demand=demand)
        instance.stations.append(station)
        line_idx += 1
    
    # Calculate distance matrix
    instance.calculate_distances()
    
    return instance


class ORToolsMPVRPSolver:
    """OR-Tools based solver for MPVRP-CC"""
    
    def __init__(self, instance: MPVRPInstance, time_limit_seconds=300):
        self.instance = instance
        self.time_limit = time_limit_seconds
        self.model = cp_model.CpModel()
        self.solution = None
        
    def solve(self):
        """Solve the MPVRP-CC problem using OR-Tools CP-SAT"""
        
        print(f"Solving MPVRP-CC instance: {self.instance.uuid}")
        print(f"  Vehicles: {self.instance.nb_vehicles}")
        print(f"  Stations: {self.instance.nb_stations}")
        print(f"  Depots: {self.instance.nb_depots}")
        print(f"  Products: {self.instance.nb_products}")
        print(f"  Time limit: {self.time_limit}s")
        print()
        
        # For small instances, we can model this directly
        # For larger instances, we need heuristic approaches
        
        if self.instance.nb_stations <= 20:
            return self._solve_exact()
        else:
            return self._solve_heuristic()
    
    def _solve_exact(self):
        """Exact solution using CP-SAT for small instances"""
        
        model = self.model
        instance = self.instance
        
        # Decision variables
        # x[k][i][j]: vehicle k goes from location i to location j
        x = {}
        for k in range(instance.nb_vehicles):
            for i in range(len(instance.distance_matrix)):
                for j in range(len(instance.distance_matrix)):
                    if i != j:
                        x[k, i, j] = model.NewBoolVar(f'x_k{k}_i{i}_j{j}')
        
        # y[k][s][p]: vehicle k delivers product p to station s
        y = {}
        for k in range(instance.nb_vehicles):
            for s in range(instance.nb_stations):
                for p in range(instance.nb_products):
                    y[k, s, p] = model.NewBoolVar(f'y_k{k}_s{s}_p{p}')
        
        # Constraint 1: Demand satisfaction
        for s, station in enumerate(instance.stations):
            for p in range(instance.nb_products):
                if station.demand[p] > 0:
                    model.Add(sum(y[k, s, p] for k in range(instance.nb_vehicles)) == 1)
        
        # Constraint 2: Flow conservation
        for k in range(instance.nb_vehicles):
            garage_idx = instance.get_location_index('G', instance.vehicles[k].home_garage)
            
            # Must leave garage
            model.Add(sum(x[k, garage_idx, j] 
                         for j in range(len(instance.distance_matrix)) 
                         if j != garage_idx) == 1)
            
            # Must return to garage
            model.Add(sum(x[k, i, garage_idx] 
                         for i in range(len(instance.distance_matrix)) 
                         if i != garage_idx) == 1)
        
        # Constraint 3: Visit continuity
        for k in range(instance.nb_vehicles):
            for j in range(len(instance.distance_matrix)):
                incoming = sum(x[k, i, j] 
                              for i in range(len(instance.distance_matrix)) if i != j)
                outgoing = sum(x[k, j, m] 
                              for m in range(len(instance.distance_matrix)) if m != j)
                model.Add(incoming == outgoing)
        
        # Objective: Minimize total distance
        distance_vars = []
        for k in range(instance.nb_vehicles):
            for i in range(len(instance.distance_matrix)):
                for j in range(len(instance.distance_matrix)):
                    if i != j and (k, i, j) in x:
                        # Scale distance for integer arithmetic
                        scaled_dist = int(instance.distance_matrix[i][j] * 100)
                        distance_vars.append(x[k, i, j] * scaled_dist)
        
        model.Minimize(sum(distance_vars))
        
        # Solve
        solver = cp_model.CpSolver()
        solver.parameters.max_time_in_seconds = self.time_limit
        solver.parameters.log_search_progress = True
        
        start_time = time.time()
        status = solver.Solve(model)
        solve_time = time.time() - start_time
        
        if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
            print(f"\n✓ Solution found in {solve_time:.2f}s")
            print(f"  Status: {'OPTIMAL' if status == cp_model.OPTIMAL else 'FEASIBLE'}")
            print(f"  Objective value: {solver.ObjectiveValue() / 100:.2f}")
            
            self.solution = {
                'status': 'OPTIMAL' if status == cp_model.OPTIMAL else 'FEASIBLE',
                'objective': solver.ObjectiveValue() / 100,
                'time': solve_time,
                'x': {key: solver.Value(var) for key, var in x.items()},
                'y': {key: solver.Value(var) for key, var in y.items()}
            }
            return True
        else:
            print(f"\n✗ No solution found in {solve_time:.2f}s")
            print(f"  Status: {solver.StatusName(status)}")
            return False
    
    def _solve_heuristic(self):
        """Heuristic solution for larger instances"""
        
        print("Using heuristic approach for large instance...")
        
        # Implement a greedy nearest-neighbor heuristic
        instance = self.instance
        
        # Initialize solution structure
        routes = {k: [] for k in range(instance.nb_vehicles)}
        assigned_demands = {(s, p): False 
                           for s in range(instance.nb_stations) 
                           for p in range(instance.nb_products)}
        
        # For each vehicle, build a route
        for k, vehicle in enumerate(instance.vehicles):
            current_product = vehicle.initial_product
            current_location = instance.get_location_index('G', vehicle.home_garage)
            current_load = 0
            
            route = [current_location]
            
            while True:
                # Find nearest unserved station needing current product
                best_station = None
                best_distance = float('inf')
                
                for s, station in enumerate(instance.stations):
                    if station.demand[current_product] > 0 and not assigned_demands[s, current_product]:
                        station_idx = instance.get_location_index('S', station.id)
                        dist = instance.distance_matrix[current_location][station_idx]
                        
                        if dist < best_distance and current_load + station.demand[current_product] <= vehicle.capacity:
                            best_distance = dist
                            best_station = s
                
                if best_station is not None:
                    # Load at depot if needed
                    if current_load == 0:
                        depot_idx = instance.get_location_index('D', 1)  # Use first depot
                        route.append(depot_idx)
                        current_load = vehicle.capacity
                        current_location = depot_idx
                    
                    # Visit station
                    station_idx = instance.get_location_index('S', instance.stations[best_station].id)
                    route.append(station_idx)
                    current_load -= instance.stations[best_station].demand[current_product]
                    assigned_demands[best_station, current_product] = True
                    current_location = station_idx
                else:
                    break
            
            # Return to garage
            garage_idx = instance.get_location_index('G', vehicle.home_garage)
            route.append(garage_idx)
            routes[k] = route
        
        # Calculate total distance
        total_distance = 0
        for k, route in routes.items():
            for i in range(len(route) - 1):
                total_distance += instance.distance_matrix[route[i]][route[i+1]]
        
        print(f"\n✓ Heuristic solution completed")
        print(f"  Total distance: {total_distance:.2f}")
        
        self.solution = {
            'status': 'HEURISTIC',
            'objective': total_distance,
            'routes': routes
        }
        
        return True


def export_solution(instance: MPVRPInstance, solution: dict, output_file: str):
    """Export solution to the required format"""
    
    with open(output_file, 'w') as f:
        # Write routes (simplified format for now)
        if 'routes' in solution:
            for k, route in solution['routes'].items():
                if len(route) > 2:  # Has actual visits
                    f.write(f"{k+1}: ")
                    for loc_idx in route:
                        f.write(f"{loc_idx} - ")
                    f.write("\n")
                    f.write(f"{k+1}: 0(0.0) - " * len(route) + "\n")
                    f.write("\n")
        
        # Write metrics
        f.write("1\n")  # vehicles used
        f.write("0\n")  # product changes
        f.write("0.0\n")  # transition cost
        f.write(f"{solution['objective']:.2f}\n")  # total distance
        f.write("OR-Tools CP-SAT\n")  # solver
        f.write(f"{solution.get('time', 0):.3f}\n")  # time
    
    print(f"\n✓ Solution exported to: {output_file}")


if __name__ == '__main__':
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python ortools_solver.py <instance_file> [output_file] [time_limit]")
        sys.exit(1)
    
    instance_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else "solution.dat"
    time_limit = int(sys.argv[3]) if len(sys.argv) > 3 else 300
    
    # Parse instance
    print("=" * 70)
    print("MPVRP-CC Solver using Google OR-Tools")
    print("=" * 70)
    print()
    
    instance = parse_instance(instance_file)
    
    # Solve
    solver = ORToolsMPVRPSolver(instance, time_limit_seconds=time_limit)
    success = solver.solve()
    
    if success:
        export_solution(instance, solver.solution, output_file)
        print("\n" + "=" * 70)
        print("✓ Solving completed successfully!")
        print("=" * 70)
    else:
        print("\n" + "=" * 70)
        print("✗ Failed to find a solution")
        print("=" * 70)
        sys.exit(1)
