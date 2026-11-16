# navigation

Travail d'Ali sur la navigation. Tout est rangé ici pour ne rien modifier dans les dossiers de l'équipe : `Codes/`, `Raspberry/`, `Ressources/` et `ros2_ws/` restent tels que l'équipe les a laissés.

## `docs/`

| Fichier | Contenu |
|---|---|
| `SLAM_2D.md` | Cours pas à pas sur le SLAM 2D (formules) et son adaptation au projet. Copie de `Ressources/SLAM_2D.md` |
| `navigation_lidar.txt` | Schéma de l'idée de navigation au LiDAR et de l'architecture ROS 2. Copie de `Idees/navigation_lidar.txt` |
| `Conception_ROS2_SLAM2D_LiDAR360_detaillee.md` | Document de conception : secteur avant → virage, secteur arrière → vitesse |
| `braquage_vitesse.md` | Cours sur les formules de braquage et de vitesse en virage, avec les valeurs réelles de la voiture |

## `codes/`

Codes Python pour comprendre les algorithmes, testés sur PC. Ce ne sont pas des nœuds ROS : ils ne sont lancés par rien et n'utilisent pas le robot. Chaque fichier a une démonstration : `python3 <fichier>.py` (nécessite `numpy`, et `scipy` pour `icp.py`).

| Fichier | Contenu | Résultat de la démonstration |
|---|---|---|
| `grille_occupation.py` | Carte en grille d'occupation (log-odds, tracé des rayons de Bresenham) | Un mur vu 3 fois : p = 0,93 sur le mur, 0,02 devant, 0,50 ailleurs |
| `icp.py` | Recalage de deux scans par ICP (solution par SVD) | Retrouve un déplacement de 5° et (8 cm, −4 cm) : 5,01° et (0,080 ; −0,040) |
| `pure_pursuit.py` | Suivi de trajectoire et conversion braquage → impulsion servo | Cercle de 2 m : courbure 0,521 (0,5 attendu), servo 1377 µs (à gauche) |

## `AU516_SLAM_Navigation/`

Copie du dossier `AU516_SLAM_Navigation` (bureau d'Ali) : une version 2.0 complète du paquet `au516_nav` avec la navigation autonome au LiDAR. Tout est expliqué dans `AU516_SLAM_Navigation/README_NAVIGATION.md`.

- **Nœuds ROS 2** : `pico_bridge` (liaison série, odométrie, TF), `local_navigator` (passages, θ_cible, commande, impasses) et `sim_2d` (simulateur).
- **Cœur de l'algorithme sans ROS** : `au516_nav/nav_core/`.
- **Lancement** : `bringup`, `robot`, `slam`, `navigation` et `sim`.
- **Réglages** : `nav.yaml`, `pico_bridge.yaml` et `slam_toolbox.yaml`, plus une configuration RViz.
- **Tests** : 13 tests de scénarios en simulation (couloir, virages, zigzag, passages étroits, obstacle, impasse, circuit en anneau, carte). Ils passent tous, relancés le 16/11 sans ROS.
- **Pas encore vérifié** : la compilation et le lancement avec ROS 2, et le comportement sur la vraie voiture. Il faut d'abord calibrer (§6 de son README).

Ce paquet porte le même nom que celui de l'équipe (`au516_nav`). Il n'est pas compilé ici grâce au `COLCON_IGNORE` de `navigation/`. Pour l'essayer, il faudra le copier sur le Pi **à la place** de `Raspberry/au516_nav`, en accord avec l'équipe.

## `corrections_au516_nav/`

Versions corrigées de fichiers du paquet `Raspberry/au516_nav`. **Elles ne sont pas utilisées** tant que personne ne les copie à leur place dans `Raspberry/au516_nav`. Chaque fichier est complet, avec le même chemin que dans le paquet.

| Fichier | Ce qui change par rapport à la version de l'équipe |
|---|---|
| `au516_nav/move.py` | Publie sur `/cmd_drive` (le topic lu par `serial_bridge`) au lieu de `/pico_cmd`, sinon le clavier ne fait pas bouger la voiture. Les touches q / d braquent du bon côté (1000 µs = gauche). |
| `setup.py` | Déclare `move` et `lidar_map` pour pouvoir les lancer avec `ros2 run`. Basé sur le `setup.py` du commit « Fixed bugs » de CouscousPL. |
| `config/slam.yaml` | Un point de carte tous les 20 cm (`minimum_travel_distance: 0.2`) au lieu de 50 cm, mieux adapté à une petite piste. |
| `package.xml` | Déclare toutes les dépendances utilisées par le code (`sensor_msgs`, `nav_msgs`, `geometry_msgs`, `tf2_ros`, `slam_toolbox`…). |
| `gitignore_racine.txt` | Proposition de `.gitignore` à mettre à la racine du repo, pour ne plus envoyer `build/`, `install/`, `log/` et `__pycache__/` sur GitHub. |

## `COLCON_IGNORE`

Fichier vide qui dit à `colcon build` d'ignorer ce dossier. Comme `corrections_au516_nav/` contient un `package.xml`, cela évite que colcon y voie un deuxième paquet `au516_nav` si quelqu'un compile le repo entier.
