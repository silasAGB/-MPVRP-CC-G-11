import sys
import json
import os
from .parser import InstanceParser
from .solver import MPVRPCCSolver


def main():
    # Utilisation d'une instance d'exemple si aucun fichier n'est fourni
    if len(sys.argv) < 2:
        print("Usage: python3 -m src.main <instance_path>")
        print("Lancement avec une instance d'exemple...")
        instance_data = InstanceParser.get_example_instance()
    else:
        instance_path = sys.argv[1]
        parser = InstanceParser(instance_path)
        instance_data = parser.parse()

    print("--- Résolution du MPVRP-CC ---")
    print(f"Camions: {instance_data['nb_trucks']}, Produits: {instance_data['nb_products']}")

    solver = MPVRPCCSolver(instance_data)
    results = solver.run(timeout=30)

    print(f"Statut: {results['status_name']}")
    if results['objective_value'] is not None:
        print(f"Coût Total: {results['objective_value']}")

        # Sauvegarde des résultats
        output_path = "results/solution.json"
        os.makedirs("results", exist_ok=True)

        # Convertir le statut en string pour la sérialisation JSON
        serializable_results = results.copy()
        serializable_results['status'] = results['status_name']

        with open(output_path, "w") as f:
            json.dump(serializable_results, f, indent=4)
        print(f"Solution sauvegardée dans {output_path}")

        for truck_info in results['routes']:
            print(f"\nCamion {truck_info['truck']}:")
            for route in truck_info['data']:
                clean_str = " (Nettoyé)" if route['cleaned'] else ""
                print(f"  Mini-route {route['mini_route']} - Produit {route['product']}{clean_str}")
                print(f"    Arcs: {route['arcs']}")


if __name__ == "__main__":
    main()
