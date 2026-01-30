# -MPVRP-CC-G-11
Implémentation du problème MPVRP-CC – Projet Master 1 IFRI Groupe 11

Ce projet implémente une solution pour le problème de routage de véhicules multiproduits avec coût de changement (MPVRP-CC) en utilisant Google OR-Tools.
Structure du Projet

    instances/ : Contient les fichiers de données des instances (format JSON).
    src/ : Code source de l'application.
        parser.py : Lecture des instances.
        model.py : Définition du modèle mathématique CP-SAT.
        solver.py : Logique de résolution et extraction des résultats.
        main.py : Point d'entrée de l'application.
    results/ : Dossier de sortie pour les solutions trouvées.

Utilisation
installation de ortools:
pip install ortools
Pour lancer la résolution :
python3 -m src.main instances/small/test_instance.json
Si aucun fichier n'est spécifié, le programme utilise une instance d'exemple intégrée.

Modélisation
La modélisation suit les équations définies dans le document modélisation.pdf :

    Minimisation des coûts de transport et de nettoyage.
    Contraintes de satisfaction de demande par station et par produit.
    Capacité des camions et restriction mono-produit par mini-route.
    Détection automatique du besoin de nettoyage entre deux mini-routes de produits différents.

