from ortools.sat.python import cp_model
from .model import MPVRPCCModel


class MPVRPCCSolver:
    def __init__(self, instance_data):
        self.instance_data = instance_data
        self.model_wrapper = MPVRPCCModel(instance_data)

    def run(self, timeout=60):
        solver, status = self.model_wrapper.solve(timeout)

        results = {
            "status": status,
            "status_name": "",
            "objective_value": None,
            "routes": []
        }

        if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
            results["status_name"] = "OPTIMAL" if status == cp_model.OPTIMAL else "FEASIBLE"
            results["objective_value"] = solver.ObjectiveValue()

            # Extraction des routes
            for k in self.model_wrapper.K:
                truck_routes = []
                for r in self.model_wrapper.R:
                    route_steps = []
                    # Simple extraction de la séquence d'arcs
                    for (i, j, truck_id, route_id) in self.model_wrapper.x:
                        if truck_id == k and route_id == r:
                            if solver.Value(self.model_wrapper.x[i, j, k, r]) == 1:
                                route_steps.append((i, j))

                    # Extraction du produit transporté
                    product = None
                    for p in self.model_wrapper.P:
                        if solver.Value(self.model_wrapper.y[r, k, p]) == 1:
                            product = p
                            break

                    if route_steps:
                        truck_routes.append({
                            "mini_route": r,
                            "product": product,
                            "arcs": route_steps,
                            "cleaned": solver.Value(self.model_wrapper.delta[k, r]) == 1
                        })
                results["routes"].append({"truck": k, "data": truck_routes})
        else:
            results["status_name"] = "NOT_FOUND"

        return results
