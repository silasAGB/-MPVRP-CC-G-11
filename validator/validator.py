import os
import json
import requests

def main():
    url = 'https://mpvrp-cc.onrender.com/model/verify'
    directory = '.'  # Dossier actuel
    results = {}
    
    # Compteurs pour les statistiques
    stats = {
        "feasible_true": 0,
        "feasible_false": 0,
        "missing_solution": 0
    }

    # 1. Récupérer la liste des fichiers qui commencent par MPVRP
    files = [f for f in os.listdir(directory) if f.startswith('MPVRP') and os.path.isfile(f)]

    print(f"--- Début du traitement ({len(files)} fichiers instances trouvés) ---")

    for instance_name in files:
        # 2. Déterminer le nom du fichier solution équivalent
        solution_name = f"Sol_{instance_name}"
        
        # Vérifier si le fichier solution existe
        if os.path.exists(os.path.join(directory, solution_name)):
            print(f"Envoi de : {instance_name} + {solution_name}...")
            
            try:
                # 3. Préparer et envoyer la requête multipart/form-data
                with open(instance_name, 'rb') as f_inst, open(solution_name, 'rb') as f_sol:
                    files_payload = {
                        'instance_file': (instance_name, f_inst),
                        'solution_file': (solution_name, f_sol)
                    }
                    
                    response = requests.post(url, files=files_payload)
                    
                    if response.status_code == 200:
                        data = response.json()
                        results[instance_name] = data
                        
                        # Mise à jour des stats
                        if data.get("feasible") is True:
                            stats["feasible_true"] += 1
                        else:
                            stats["feasible_false"] += 1
                    else:
                        print(f"Erreur API ({response.status_code}) pour {instance_name}")
                        results[instance_name] = {"error": "API_ERROR", "status_code": response.status_code}
            
            except Exception as e:
                print(f"Erreur lors du traitement de {instance_name}: {e}")
                results[instance_name] = {"error": str(e)}
        
        else:
            # Si le fichier solution n'existe pas
            print(f"Solution manquante pour : {instance_name}")
            results[instance_name] = None
            stats["missing_solution"] += 1

    # 4. Enregistrer tout dans un fichier JSON
    output_filename = "resultats_verification.json"
    with open(output_filename, 'w', encoding='utf-8') as f_out:
        json.dump(results, f_out, indent=4, ensure_ascii=False)

    # 5. Afficher le résumé final
    print("\n" + "="*30)
    print("RÉSUMÉ FINAL")
    print("="*30)
    print(f"Total fichiers instances : {len(files)}")
    print(f"Solutions 'Feasible' (True)  : {stats['feasible_true']}")
    print(f"Solutions 'Feasible' (False) : {stats['feasible_false']}")
    print(f"Fichiers sans solution      : {stats['missing_solution']}")
    print(f"Détails enregistrés dans     : {output_filename}")

if __name__ == "__main__":
    main()
