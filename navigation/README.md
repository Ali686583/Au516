# navigation

Travail d'Ali sur la navigation. Tout est rangé ici pour ne rien modifier dans les dossiers de l'équipe : `Codes/`, `Raspberry/`, `Ressources/` et `ros2_ws/` restent tels que l'équipe les a laissés.

## Intégration dans le paquet du robot (16/11, branche `integration-navigation`)

La navigation autonome est maintenant **intégrée dans `Raspberry/au516_nav`**, le paquet que `push_to_pi.ps1` envoie sur le Pi. Mode d'emploi complet : [`Raspberry/au516_nav/README.md`](../Raspberry/au516_nav/README.md). On lance tout avec `ros2 launch au516_nav robot_launch.py`.

| Venu de `navigation/AU516_SLAM_Navigation/au516_nav/` | Dans `Raspberry/au516_nav/` |
|---|---|
| `au516_nav/nav_core/` (algorithme) | copié, avec en plus `nav_core/drive.py` (conversion `[speed, pulse]`) |
| `local_navigator.py` | copié, avec ces changements : état publié sur `/nav/enabled`, **correction d'un plantage** (couleurs entières dans les marqueurs RViz, refusées par ROS 2 Jazzy), arrêt propre avec CTRL+C |
| `sim_2d.py` | copié, avec en plus l'émulation du Pico sur un faux port série et `/sim/ground_truth` |
| `config/nav.yaml`, `rviz/au516_nav.rviz` | `config/nav.yaml` (`autostart: false`), `config/navigation.rviz` |
| `test/test_scenarios.py`, `test/test_map_memory.py` | copiés tels quels |
| `pico_bridge.py`, `config/pico_bridge.yaml` | **non repris** : ils faisaient double emploi avec `serial_bridge` et `odometry_node` (même port série, même `/odom`). La conversion `/cmd_vel` → `[speed, pulse]` est dans le nouveau nœud `drive_mux` |
| `launch/*.launch.py` | **non repris**, remplacés par `launch/robot_launch.py`. Le `slam.launch.py` d'ici lançait slam_toolbox comme un nœud simple : sous Jazzy, il n'est alors jamais activé (pas de carte). `robot_launch.py` utilise le même lancement que `slam_launch.py` de l'équipe |
| `encoder_reader.py`, `conversion_2D.py`, `scan_viewer.py` | non repris : les versions de l'équipe sont déjà dans le paquet |

Ce dossier `navigation/` reste tel quel, comme référence. Il est toujours ignoré par colcon (`COLCON_IGNORE`).

## `README_NAVIGATION.md`

Le mode d'emploi complet du paquet de navigation autonome (`AU516_SLAM_Navigation/`) : architecture, algorithme pas à pas, installation, lancement, calibration, tests et limites. Mis à jour le 16/11 avec les vrais réglages de la voiture. Les deux autres copies, dans `AU516_SLAM_Navigation/` et `AU516_SLAM_Navigation/au516_nav/`, sont identiques.

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

Copie du dossier `AU516_SLAM_Navigation` (bureau d'Ali), avec les réglages corrigés (voir plus bas) : une version 2.0 complète du paquet `au516_nav` avec la navigation autonome au LiDAR. Tout est expliqué dans `AU516_SLAM_Navigation/README_NAVIGATION.md`.

- **Nœuds ROS 2** : `pico_bridge` (liaison série, odométrie, TF), `local_navigator` (passages, θ_cible, commande, impasses) et `sim_2d` (simulateur).
- **Cœur de l'algorithme sans ROS** : `au516_nav/nav_core/`.
- **Lancement** : `bringup`, `robot`, `slam`, `navigation` et `sim`.
- **Réglages** : `nav.yaml`, `pico_bridge.yaml` et `slam_toolbox.yaml`, plus une configuration RViz.
- **Tests** : 13 tests de scénarios en simulation (couloir, virages, zigzag, passages étroits, obstacle, impasse, circuit en anneau, carte). Ils passent tous, relancés le 16/11 sans ROS.
- **Pas encore vérifié** : la compilation et le lancement avec ROS 2, et le comportement sur la vraie voiture. Il faut d'abord calibrer (§6 de son README).

**Réglages corrigés** (16/11) avec les valeurs de la voiture tirées de `odometry_node.py` et `Commands list` de l'équipe. Les 13 tests passent toujours avec ces valeurs.

| Réglage | Avant | Après | Fichiers |
|---|---|---|---|
| Empattement `wheelbase` | 0,25 m | **0,165 m** | `nav.yaml`, `pico_bridge.yaml`, `params.py`, `sim_2d.py`, `pico_bridge.py` |
| Braquage max `max_steer` | 0,45 rad (26°) | **0,34907 rad (20°)** | les mêmes |
| `servo_us_at_max_steer` | 450 µs | **500 µs** (1500 ± 500 = ±20°) | `pico_bridge.yaml`, `pico_bridge.py` |
| `steering_sign` | 1.0 | **-1.0** : avant, un virage à gauche envoyait plus de 1500 µs, donc braquait à droite | `pico_bridge.yaml`, `pico_bridge.py` |

Encore à mesurer sur la voiture : `laser_x` (position du LiDAR), `track`, `width`, `front_overhang`, `rear_overhang`.

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
