# MPVRP-CC Solver 

**Multi-Product Vehicle Routing Problem with Changeover Cost**

> Projet académique & ingénierie logicielle — Solveur modulaire en Python respectant les spécifications officielles MPVRP-CC

---

## Présentation

Ce projet implémente un **solveur modulaire et extensible** pour le problème **MPVRP-CC (Multi-Product Vehicle Routing Problem with Changeover Cost)**.

Le MPVRP-CC est une généralisation du VRP classique qui prend en compte :

* La distribution de **plusieurs types de produits**
* La contrainte **un seul produit transporté à la fois par véhicule**
* Un **coût de nettoyage (changeover cost)** lors du changement de produit

Ce projet est conçu pour :

*  Projets académiques (Master, recherche opérationnelle, data science)
*  Expérimentation algorithmique
*  Cas industriels (logistique pétrolière, chimique, alimentaire)

---

## Objectifs

* Parser des fichiers d’instances `.dat` conformes à la spécification MPVRP-CC
* Construire des routes valides respectant toutes les contraintes
* Minimiser :

  * La **distance totale parcourue**
  * Le **coût total de changement de produit**
* Exporter une solution valide au **format officiel MPVRP-CC**
* Fournir une base pour intégrer des **métaheuristiques avancées**

---

##  Fonctionnalités

* Parser officiel des instances `.dat`
* Modèles de données (véhicules, dépôts, garages, stations, produits)
* Calcul des distances euclidiennes
* Heuristique gloutonne (baseline)
* Gestion du coût de changement de produit
* Validation des contraintes
* Export du fichier solution `.dat`
* Visualisation des routes (optionnel)
* Architecture prête pour :

  * Tabu Search
  * Algorithmes génétiques
  * Large Neighborhood Search (LNS)

---

## Architecture du projet

```
mpvrp-cc-solver/
│
├── src/
│   ├── parser.py        # Lecture des fichiers d’instance
│   ├── models.py       # Structures de données principales
│   ├── distance.py    # Calcul des distances
│   ├── heuristics.py  # Heuristiques de construction
│   ├── cost.py        # Gestion des coûts de changement
│   ├── validator.py  # Vérification des contraintes
│   ├── exporter.py   # Génération du fichier solution
│   └── visualizer.py # Affichage graphique des routes
│
├── data/
│   ├── instances/     # Fichiers .dat d’instances
│   └── solutions/    # Solutions générées
│
├── main.py            # Point d’entrée du solveur
├── requirements.txt
├── .gitignore
└── README.md
```

---

## Installation

### 1. Cloner le projet

```bash
git clone https://github.com/silasAGB/mpvrp-cc-solver.git
cd mpvrp-cc-solver
```

### 2. Environnement Python

Recommandé : Python 3.9+

Créer un environnement virtuel :

```bash
python -m venv venv
source venv/bin/activate
```

Installer les dépendances :

```bash
pip install -r requirements.txt
```

---

## Organisation des instances par difficulté

Les instances sont classées selon leur niveau de complexité afin de faciliter les tests, le benchmarking et la comparaison des heuristiques.

```
data/instances/
├── small/    # Petites instances (tests rapides, débogage)
├── medium/   # Instances intermédiaires (évaluation des heuristiques)
└── large/    # Grandes instances (stress test, performance, recherche)
```

### Convention de nommage

* `MPVRP_S_XXX.dat` → Small
* `MPVRP_M_XXX.dat` → Medium
* `MPVRP_L_XXX.dat` → Large

Cette organisation permet d’évaluer la **scalabilité** des algorithmes et d’analyser l’impact de la taille du problème sur :

* Le temps d’exécution
* La qualité des solutions
* Le coût total (distance + changeover)

---

Le coût total (distance + changeover)
