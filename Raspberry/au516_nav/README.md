# au516_nav — paquet ROS 2 du robot AU516

Liaison avec le Pico, odométrie, SLAM, navigation autonome au LiDAR et conduite au clavier, dans **un seul paquet** et **un seul lancement**.
La navigation autonome vient du dossier `navigation/AU516_SLAM_Navigation/` du repo. Elle est maintenant branchée sur les nœuds de l'équipe (`serial_bridge`, `odometry_node`, `move`, `Lidar_map`) au lieu de les remplacer.

## Lancer le robot

Sur le Pi, après `colcon build --packages-select au516_nav` puis `source install/setup.bash` :

```bash
ros2 launch au516_nav robot_launch.py
```

Dans un deuxième terminal, pour le clavier :

```bash
ros2 run au516_nav move
```

- **Au départ**, la navigation est en attente et la voiture ne bouge pas.
- **Au clavier** : `z` / `s` pour la vitesse, `q` / `d` pour la direction, `c` pour centrer, `espace` pour s'arrêter.
- **Touche `n`** : la navigation autonome prend la main.
- **Reprise en main** : n'importe quelle touche de conduite (`z s q d c espace`) arrête la navigation et rend la main au clavier, en repartant d'une voiture arrêtée.
- **Sans `move`**, on peut démarrer et arrêter la navigation avec :
  ```bash
  ros2 topic pub --once /nav/enable std_msgs/msg/Bool "{data: true}"
  ```
  et `{data: false}` pour l'arrêter.
- **Vue de dessus du LiDAR** (page web de l'équipe) : `http://192.168.5.1:8000`.
- **Vue RViz sur le PC** :
  ```bash
  ros2 launch au516_nav display_launch.py config:=navigation.rviz
  ```

### Arguments utiles

| Argument | Défaut | Rôle |
|---|---|---|
| `sim` | `false` | `true` : robot simulé (voir plus bas) |
| `navigation` | `true` | `false` : pas de navigation, `move` pilote `serial_bridge` directement, comme avant |
| `autostart` | `false` | `true` : la navigation roule dès le lancement |
| `v_max` | `0.6` | vitesse max de la navigation en m/s (la voiture plafonne vers 0,81). **Commencer à `0.3`** |
| `slam` | `true` | lance slam_toolbox avec `config/slam.yaml` |
| `laser_x`, `laser_y`, `laser_z`, `laser_yaw` | `0.22`, `0`, `0.10`, `0` | position du LiDAR par rapport à l'essieu arrière. **À mesurer.** Les mêmes valeurs servent à la TF `base_link → laser` (SLAM) et à la navigation. `laser_yaw:=3.14159` si le LiDAR est monté à l'envers |
| `lidar_port`, `lidar_scan_mode`, `pico_port` | ports `by-id` de l'équipe, `DenseBoost` | ports série |
| `web_viewer`, `web_port` | `true`, `8000` | page web `Lidar_map` |
| `rviz` | `false` | ouvre RViz (seulement s'il y a un écran) |

Premier essai conseillé, roues en l'air :

```bash
ros2 launch au516_nav robot_launch.py v_max:=0.3
```

puis `n` dans `move` : les roues doivent tourner et braquer du côté libre.

### Sans le robot (simulation)

```bash
ros2 launch au516_nav robot_launch.py sim:=true autostart:=true scenario:=ring
```

- **Ce qui est simulé** : `sim_2d` joue le rôle du **matériel**. Il publie `/scan` comme le RPLIDAR, et il crée un faux port série (`/tmp/au516_fake_pico`) qui répond exactement comme `Codes/main.py` sur le Pico :
  - il reçoit `C <speed> <pulse>` ;
  - il applique le chien de garde de 500 ms ;
  - il renvoie `E <enc_G> <enc_D>` toutes les 20 ms.
- **Ce qui tourne pour de vrai** : tous les autres nœuds sont ceux du robot, sans modification (`serial_bridge`, `odometry_node`, `drive_mux`, slam_toolbox, `local_navigator`, `lidar_map`).
- **Scénarios** : `ring` (anneau avec chicane), `A_straight`, `B_left`, `C_right`, `D_zigzag`, `E_narrow_ok`, `E_narrow_refused`, `F_obstacle`, `F_blocked`, `G_dead_end`, `T_junction`.
- **Topics propres au simulateur** : `/sim/collision`, `/sim/ground_truth` (vraie position) et `/sim/walls` (murs pour RViz).

## Architecture

```
 RPLIDAR C1 ──/scan──┬──────────────► slam_toolbox ──/map, TF map→odom──┐
 (rplidar_ros)       │                                                  ▼
                     └────────────────/scan──────────────────────► local_navigator
                                                                        │ /cmd_vel (v, ω)
                                                                        ▼
 move (clavier) ──/cmd_drive [speed, pulse]──────────────────────► drive_mux ◄──/nav/enabled
                                                                        │ /cmd_drive_mux [speed, pulse]
                                                                        ▼
 Pico (Codes/main.py) ◄──série "C speed pulse"── serial_bridge ──/encoders, /steering_pwm──► odometry_node
                      ──série "E encG encD"────►                          /odom, TF odom→base_link ◄┘
```

| Nœud | Origine | Rôle |
|---|---|---|
| `serial_bridge` | équipe, **inchangé** | seul nœud qui ouvre le port du Pico. Dans `robot_launch.py`, son entrée `/cmd_drive` est remappée sur `/cmd_drive_mux` |
| `odometry_node` | équipe, **inchangé** | `/odom` et TF `odom → base_link` à partir des encodeurs et du servo |
| `move` | équipe, complété | clavier. Ajouts : touche `n` et reprise en main (voir plus haut) |
| `lidar_map`, `scan_viewer`, `conversion_2D` | équipe, **inchangés** | visualisation et outils |
| `local_navigator` | `navigation/`, adapté | passages traversables → θ_cible → `/cmd_vel`. Utilise `/map` et la pose SLAM si elles existent. Publie aussi son état sur `/nav/enabled` (latched) |
| `drive_mux` | **nouveau** | choisit la source de commande. **MANUEL** : le clavier passe tel quel. **AUTO** : `/cmd_vel` est converti en `[speed, pulse]` et le clavier est ignoré. Envoie un arrêt `[0, 1500]` à chaque changement de mode, et un arrêt si `/cmd_vel` manque plus de 0,3 s |
| `sim_2d` | `navigation/`, complété | simulateur. Ajouts : émulation du Pico sur un faux port série, et `/sim/ground_truth` |

### Conversion de la commande (`nav_core/drive.py`)

| Grandeur | Formule ou valeur | Source |
|---|---|---|
| `speed` | `round(v / k_v)`, borné à [-10, 50] | `MotorDriverI2C.clamp` |
| `k_v` | `100·π·0,065 / 1260 ≈ 0,0162` m/s par unité, soit une vitesse max d'environ 0,81 m/s | — |
| `pulse` | `1500 − 500·δ/δmax`, borné à [1000, 2000] | — |
| `δmax` | 20°. 1000 µs = 20° à gauche, 2000 µs = 20° à droite | `Commands list` |
| `δ` | `atan(L·ω / v)` (modèle bicyclette), avec `L = 0,165` m | — |

- `odometry_node` retransforme l'impulsion en angle avec exactement la même formule : `test/test_drive.py` vérifie que les deux restent cohérents.
- Si quelqu'un change les valeurs de `odometry_node.py`, ce test échoue tant que `nav_core/drive.py` et `config/nav.yaml` n'ont pas été mis à jour.

### Sécurités

| Situation | Réaction | Testé |
|---|---|---|
| LiDAR perdu plus de 0,5 s | `local_navigator` passe en `SAFE_STOP` et envoie une vitesse nulle | oui |
| Navigation muette plus de 0,3 s (mode AUTO) | `drive_mux` envoie un arrêt | oui |
| `/cmd_vel` invalide (NaN) | arrêt | oui |
| Plus de commande sur `serial_bridge` | il envoie `C 0 1500` après 0,3 s | oui |
| Liaison série coupée | le Pico s'arrête au bout de 500 ms | oui, arrêt mesuré à 0,55 s |
| Un nœud plante | `respawn` le relance au bout de 2 s ; avec un CTRL+C normal, rien n'est relancé | oui |
| Clavier pendant la navigation | n'importe quelle touche de conduite coupe la navigation | oui |

## Tests

```bash
colcon test --packages-select au516_nav && colcon test-result --verbose
```

Ou sans colcon : `python3 -m pytest test/`.

| Fichier | Ce qu'il vérifie | ROS nécessaire |
|---|---|---|
| `test_scenarios.py` | navigation en boucle fermée sur les scénarios A à G et l'anneau | non |
| `test_map_memory.py` | carte SLAM et mémoire des impasses | non |
| `test_drive.py` | cohérence entre la conversion `[speed, pulse]`, `odometry_node.py` et `nav.yaml` | non |
| `test_integration_ros.py` | les vrais nœuds ensemble : `drive_mux`, `serial_bridge` sur un faux port série avec le Pico émulé, `odometry_node`, `local_navigator` | oui |
| `test_flake8.py`, `test_pep257.py`, `test_copyright.py` | style (modèles ROS) | — |

Les tests de style échouaient déjà avant l'intégration, à cause du style des fichiers de l'équipe (guillemets doubles, lignes longues). Les nouveaux fichiers passent flake8 sans erreur.

## Réglages (`config/`)

| Fichier | Rôle |
|---|---|
| `slam.yaml` | slam_toolbox, celui de l'équipe, utilisé tel quel par `slam_launch.py` et `robot_launch.py` |
| `nav.yaml` | `local_navigator` : géométrie de la voiture et paramètres de la navigation (marges, vitesses, demi-tour…) |
| `slam.rviz`, `navigation.rviz` | vues RViz (carte seule, ou carte + passages + θ_cible + trajectoire) |

**À mesurer sur la voiture** :
- dans `nav.yaml` : `laser_x` (et les arguments `laser_*` du lancement), `track`, `width`, `front_overhang`, `rear_overhang` ;
- sur la piste : vérifier que les tuyaux de VMC sont bien vus par le LiDAR.

## Limites connues

- **Testé** dans une machine Linux Ubuntu 24.04 avec ROS 2 Jazzy, comme le Pi, avec le LiDAR et le Pico simulés.
- **Pas testé** : sur la vraie voiture, et avec le vrai pilote `rplidar_ros` (absent de la machine de test). La composition du lancement en mode réel a été vérifiée sans l'exécuter.
- **Demi-tour en impasse** : réussi dans 5 essais complets sur 6 en simulation. Le sixième s'est terminé par l'arrêt de sécurité (`STOPPED`), sans collision.
- **Pas encore fait** : le SLAM sert à la pose et à la mémoire, mais il n'y a pas de trajectoire de course optimisée pour les tours suivants.
- **Pas encore fait** : les deux moteurs reçoivent toujours la même consigne (pas de différentiel électronique). `/nav/wheel_speeds_ideal` montre seulement les vitesses théoriques.
