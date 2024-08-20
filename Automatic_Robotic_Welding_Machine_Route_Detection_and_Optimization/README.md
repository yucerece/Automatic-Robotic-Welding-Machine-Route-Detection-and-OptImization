# Welding Path Planning and Execution with RoboDK and OCC

## English
This project aims to perform welding path planning and execution using RoboDK and the OpenCASCADE (OCC) libraries. It involves the following key steps:
1. **Reading and Processing CAD Data**: Parsing and analyzing STEP files to identify and extract relevant geometric information.
2. **Path Planning with A star Algorithm**: Using the A* algorithm for 3D path planning to determine the shortest collision-free path.
3. **Collision Detection and Angle Adjustment**: Adjusting robot angles to avoid collisions during welding operations.
4. **Executing Welding Paths**: Using RoboDK to move the robot along the planned paths for welding.

## Requirements

- Python 3.x
- RoboDK
- OpenCASCADE (OCC) libraries (PythonOCC)
- NumPy
- Pandas
- Tkinter

## Installation

1. Install Python 3.x from [Python.org](https://www.python.org/).
2. Install RoboDK and ensure it's properly set up on your system.
3. Install the required Python libraries:
    ```bash
    pip install numpy pandas matplotlib openpyxl
    ```
4. Install PythonOCC:
    ```bash
    pip install pythonocc-core
    ```

## File Structure

- `etape1.py`: Main script for reading and processing the CAD data, setting up curves, and detecting collisions.
- `etape3.py`: Script for executing welding paths using the A* algorithm and RoboDK.
- `functions.py`: Contains helper functions for processing CAD data, detecting collisions, and calculating angles.
- `interface.py`: Tkinter-based GUI for selecting and testing curves for welding.
- `astar.py` class (in `etape1.py` and `etape3.py`): Implements the A* algorithm for 3D path planning.
- `README.md`: This file, providing an overview of the project.

## Usage

### Running the Interface

To run the interface for selecting and testing curves:
```bash
python interface.py
```

## Function Descriptions
1. etape1.py
- Reads and processes the CAD data.
- Sets up curves and detects intersections.
- Uses functions from `functions.py` to determine paths and angles for welding.

2. etape3.py
- Executes the welding paths using the A* algorithm and RoboDK.
- Uses data from processed curves and performs welding operations.

3. functions.py
- `set_curves()`: Reads and processes the CAD data to extract curves and intersections.
- `create_curves(curveList)`: Adds the detected curves to RoboDK.
- `split_solid0_into_parts(mainfaces)`: Divides a solid0 into parts and returns the points for each part.
- `interpolate_points(start, end, step_size=1.0)`: Generates interpolated points between two points.
- `points_of_each_solid(solids)`: Returns the points for each solid object.
- `points_of_each_face(mainfaces)`: Returns the points for each face.
- `find_way(curveList, curve)`: Determines the axis on which the curve is located.
- `rotation_angles(intersectSolid1, intersectSolid2, curve, curveList, grupFace, solid0Liste)`: Calculates the rotation angles for approaching the curve.
- `testCollision(x_range, y_range, z_range, x_ref, y_ref, z_ref, rx, ry, rz, step)`: Checks for collisions and returns the coordinates and angles that avoid collisions.

4. functions.py
- Provides a GUI for selecting and testing curves.
- Uses Tkinter for the interface and subprocess for running scripts.

5. astar.py
- `__init__(self, start, goal, obstacles, resolution=0.5)`: Initializes the A* algorithm with start and goal points, obstacles, and resolution.
- `heuristic_fun(self, node)`: Heuristic function to estimate the cost between the current node and the goal.
- `get_neighbors(self, node)`: Retrieves the neighboring nodes of the current node.
- `is_collide(self, point)`: Checks if a point collides with any of the obstacles.
- `run(self, N=None)`: Runs the A* algorithm to find the shortest path.
- `path(self)`: Constructs the path from the start point to the goal.
- `visualization(self)`: Visualizes the path and obstacles (optional).

## References
This code is adapted from the A* 3D path planning implementation by zhm-real. The original implementation can be found at:
https://github.com/zhm-real/PathPlanning/blob/master/Search_based_Planning/Search_3D/Astar3D.py

## Français
Ce projet vise à effectuer la planification et l'exécution de trajectoires de soudage en utilisant RoboDK et les bibliothèques OpenCASCADE (OCC). Il comprend les étapes clés suivantes :
1. **Lecture et Traitement des Données CAO**: Analyse et extraction des informations géométriques pertinentes à partir des fichiers STEP.
2. **Planification de Trajectoire avec l'Algorithme A étoile**: Utilisation de l'algorithme A* pour la planification de trajectoire en 3D afin de déterminer le chemin le plus court sans collision.
3. **Détection des Collisions et Ajustement des Angles**: Ajustement des angles du robot pour éviter les collisions pendant les opérations de soudage.
4. **Exécution des Trajectoires de Soudage**: Utilisation de RoboDK pour déplacer le robot le long des trajectoires planifiées pour le soudage.

## Exigences

- Python 3.x
- RoboDK
- OpenCASCADE (OCC) libraries (PythonOCC)
- NumPy
- Pandas
- Tkinter

## Installation

1. Installez Python 3.x depuis Python.org.
2. Installez RoboDK et assurez-vous qu'il est correctement configuré sur votre système.
3. Installez les bibliothèques Python requises:
    ```bash
    pip install numpy pandas matplotlib openpyxl
    ```
4. Installez PythonOCC:
    ```bash
    pip install pythonocc-core
    ```

## Structure des Fichiers

- `etape1.py`: Script principal pour la lecture et le traitement des données CAO, la configuration des courbes et la détection des collisions.
- `etape3.py`: Script pour exécuter les trajectoires de soudage en utilisant l'algorithme A* et RoboDK.
- `functions.py`: Contient des fonctions d'assistance pour le traitement des données CAO, la détection des collisions et le calcul des angles.
- `interface.py`: Interface graphique basée sur Tkinter pour la sélection et le test des courbes pour le soudage.
- `astar.py` class (dans etape1.py et etape3.py) : Implémente l'algorithme A* pour la planification de trajectoire en 3D.
- `README.md`: Ce fichier, fournissant un aperçu du projet.

## Utilisation

### Exécution de l'Interface

Pour exécuter l'interface de sélection et de test des courbes :
```bash
python interface.py
```

## Descriptions des Fonctions
1. etape1.py
- Lit et traite les données CAO.
- Configure les courbes et détecte les intersections.
- Utilise les fonctions de functions.py pour déterminer les trajectoires et les angles pour le soudage.

2. etape3.py
- Exécute les trajectoires de soudage en utilisant l'algorithme A* et RoboDK.
- Utilise les données des courbes traitées et effectue les opérations de soudage.

3. functions.py
- `set_curves()`: Lit et traite les données CAO pour extraire les courbes et les intersections.
- `create_curves(curveList)`: Ajoute les courbes détectées à RoboDK.
- `split_solid0_into_parts(mainfaces)`: Divise un solid0 en parties et renvoie les points de chaque partie.
- `interpolate_points(start, end, step_size=1.0)`: Génère des points interpolés entre deux points.
- `points_of_each_solid(solids)`: Renvoie les points pour chaque objet solide.
- `points_of_each_face(mainfaces)`: Renvoie les points pour chaque face.
- `find_way(curveList, curve)`: Détermine l'axe sur lequel se trouve la courbe.
- `rotation_angles(intersectSolid1, intersectSolid2, curve, curveList, grupFace, solid0Liste)`: Calcule les angles de rotation pour aborder la courbe.
- `testCollision(x_range, y_range, z_range, x_ref, y_ref, z_ref, rx, ry, rz, step)`: Vérifie les collisions et renvoie les coordonnées et les angles qui évitent les collisions.

4. functions.py
- Fournit une interface graphique pour la sélection et le test des courbes.
- Utilise Tkinter pour l'interface et subprocess pour exécuter les scripts.

5. astar.py
- `__init__(self, start, goal, obstacles, resolution=0.5)`: Initialise l'algorithme A* avec les points de départ et d'arrivée, les obstacles et la résolution.
- `heuristic_fun(self, node)`: Fonction heuristique pour estimer le coût entre le nœud actuel et l'objectif.
- `get_neighbors(self, node)`: Récupère les nœuds voisins du nœud actuel.
- `is_collide(self, point)`: Vérifie si un point entre en collision avec un obstacle.
- `run(self, N=None)`: Exécute l'algorithme A* pour trouver le chemin le plus court.
- `path(self)`: Construit le chemin du point de départ à l'objectif.
- `visualization(self)`: Visualise le chemin et les obstacles (optionnel).

## Références
Ce code est adapté de l'implémentation de la planification de trajectoire 3D avec l'algorithme A* par zhm-real. L'implémentation originale peut être trouvée à l'adresse :
https://github.com/zhm-real/PathPlanning/blob/master/Search_based_Planning/Search_3D/Astar3D.py

