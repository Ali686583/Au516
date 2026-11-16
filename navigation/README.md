# navigation

Travail d'Ali sur la navigation. Tout est rangé ici pour ne rien modifier dans les dossiers de l'équipe : `Codes/`, `Raspberry/`, `Ressources/` et `ros2_ws/` restent tels que l'équipe les a laissés.

## `docs/`

| Fichier | Contenu |
|---|---|
| `SLAM_2D.md` | Cours pas à pas sur le SLAM 2D (formules) et son adaptation au projet |
| `navigation_lidar.txt` | Schéma de l'idée de navigation au LiDAR et de l'architecture ROS 2 |
| `Conception_ROS2_SLAM2D_LiDAR360_detaillee.md` | Document de conception : secteur avant → virage, secteur arrière → vitesse |

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
