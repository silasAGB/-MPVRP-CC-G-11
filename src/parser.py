import json
import os


class InstanceParser:
    """
    Parser pour les instances du problème MPVRP-CC.
    On suppose un format JSON pour la flexibilité, mais adaptable selon les fichiers réels.
    """

    def __init__(self, file_path):
        self.file_path = file_path
        self.data = {}

    def parse(self):
        if not os.path.exists(self.file_path):
            raise FileNotFoundError(f"Fichier d'instance non trouvé : {self.file_path}")

        with open(self.file_path, 'r') as f:
            self.data = json.load(f)

        return self.data

    @staticmethod
    def get_example_instance():
        """Génère une petite instance d'exemple pour le test."""
        return {
            "nb_trucks": 2,
            "nb_products": 2,
            "nb_mini_routes": 3,
            "changeover_cost": 50,
            "truck_capacity": [100, 100],
            "nodes": {
                "garages": [0],
                "depots": [1],
                "stations": [2, 3]
            },
            "demands": {
                "2": {"0": 30, "1": 20},
                "3": {"0": 15, "1": 40}
            },
            "distances": [
                [0, 10, 20, 25],
                [10, 0, 15, 18],
                [20, 15, 0, 5],
                [25, 18, 5, 0]
            ]
        }
