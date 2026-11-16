# AU516 — SLAM Toolbox + navigation autonome LiDAR

Package ROS 2 (Jazzy, Ubuntu 24.04, RPi5) `au516_nav`, version 2.0.
Référence fonctionnelle : `Idees/navigation_lidar.txt`.

## 0. Ce qui est vérifié, et ce qui ne l'est pas

| Élément | Statut |
|---|---|
| Algorithme de navigation (passages, θ_cible, contrôle, impasses) | **Testé** en boucle fermée dans un simulateur 2D Python : 13 tests, tous les scénarios A–G + circuit en anneau, 0 collision (`test/`). |
| Nœuds ROS 2 (`pico_bridge`, `local_navigator`, `sim_2d`) | **Testés avec un ROS factice** (rclpy simulé) : câblage topics/TF, commandes série `C speed pulse`, odométrie, watchdog. |
| `colcon build`, lancement réel ROS 2, SLAM Toolbox, RViz2 | **NON vérifiés** : ROS 2 n'était pas installable dans l'environnement de développement. À faire en premier sur le RPi5 (§5). |
| Comportement sur la vraie voiture | **NON vérifié**. La calibration (§6) est obligatoire avant de rouler. |

## 1. Analyse du projet existant

**Matériel (sujet AU516)** : RPi5 + ROS 2, Pico W, driver moteur I2C, 2 moteurs 12 V avec encodeurs, **servomoteur de direction**, RPLIDAR C1 à l'avant.

**Point clé : la voiture est de type Ackermann, pas différentielle.**
`Codes/motor_driver.py::set_speed()` envoie la même consigne aux deux moteurs (`[-speed, 0, speed, 0]`), et la direction est donnée par le servo (`set_steering`, 1000–2000 µs). Les formules `V_G = v − ωL/2`, `V_D = v + ωL/2` ne peuvent donc pas piloter ce robot. La commande passe par le **modèle bicyclette** :

```
ω = v·tan(δ)/L   ⇔   δ = atan(L·ω / v)        L = empattement
```

Les vitesses `V_G`/`V_D` sont tout de même calculées et publiées (`/nav/wheel_speeds_ideal`) pour le diagnostic : elles respectent `V_G < V_D` à gauche et `V_G > V_D` à droite (vérifié par les tests B et C).

| Existant | Rôle | Ce qui manquait |
|---|---|---|
| `Codes/main.py` (Pico) | reçoit `C <speed> <steering_us>`, renvoie `E <encG> <encD>` toutes les 20 ms, watchdog 500 ms | rien : **inchangé** |
| `au516_nav/encoder_reader.py` | lit les encodeurs, publie `/encoders` | aucun nœud n'**envoyait** les commandes ; pas de `/odom` ni de TF `odom → base_link` |
| `au516_nav/scan_viewer.py`, `conversion_2D.py` | affichage / conversion du scan | conservés tels quels |
| `rplidar_ros` | publie `/scan` (frame `laser`) | pas de TF `base_link → laser` |
| SLAM / navigation / `/cmd_vel` | — | tout était à créer |

Compatibilité SLAM Toolbox : il lui faut `/scan` + TF `odom → base_link`. Le nouveau `pico_bridge` fournit cette TF, ce qui rend la chaîne complète : `map → odom → base_link → laser`.

## 2. Architecture finale

```
 RPLIDAR C1 ──/scan──┬───────────────► SLAM Toolbox ──/map + TF map→odom──┐
 (rplidar_ros)       │                     ▲                               │
                     │           TF odom→base_link                         ▼
                     │                     │                    ┌──────────────────────┐
 Pico W ◄─UART─► pico_bridge ──/odom───────┘                    │   local_navigator    │
   ▲  "C speed us"   ▲  "E encG encD"                           │ 1 analyse géométrique│
   │                 │                                          │ 2 passages traversab.│
 servo + moteurs     └──────────── /cmd_vel (v, ω) ◄────────────│ 3 θ_cible            │
                                                                │ 4 contrôle v, ω      │
 TF statique base_link→laser                                    │ 5 impasses + carte   │
                                                                └──────────────────────┘
                                         /nav/markers /nav/path /nav/theta_target /nav/state → RViz2
```

| N° demandé | Composant | Fichier |
|---|---|---|
| 1 | Acquisition LiDAR | `rplidar_ros` (existant), `launch/robot.launch.py` |
| 2 | Odométrie + TF | `pico_bridge.py` (encodeurs + modèle bicyclette), TF statique laser |
| 3 | SLAM Toolbox | `config/slam_toolbox.yaml`, `launch/slam.launch.py` |
| 4–6 | Analyse locale, passages, θ_cible | `nav_core/geometry.py`, `nav_core/navigator.py` |
| 7 | Navigation globale (légère) | `nav_core/mapping.py` : carte SLAM + mémoire d'impasses |
| 8 | Contrôle du mouvement | `nav_core/control.py` |
| 9 | Interface roues | `pico_bridge.py` → Pico |
| 10 | Visualisation | `rviz/au516_nav.rviz`, marqueurs de `local_navigator` |

**Un seul éditeur de `/cmd_vel`** : `local_navigator`. **Un seul propriétaire du port série** : `pico_bridge` (ne pas lancer `encoder_reader` en même temps ; `pico_bridge` republie `/encoders` pour rester compatible).

SLAM Toolbox ne sert qu'à la pose et à la carte. Il ne planifie rien et n'évite rien.

### Topics

| Topic | Type | Émetteur |
|---|---|---|
| `/scan` | LaserScan | rplidar_ros (ou sim_2d) |
| `/odom`, TF `odom→base_link` | Odometry | pico_bridge (ou sim_2d) |
| `/map`, TF `map→odom` | OccupancyGrid | slam_toolbox |
| `/cmd_vel` | Twist | local_navigator |
| `/nav/theta_target` | Float32 (degrés) | local_navigator |
| `/nav/state` | String | local_navigator |
| `/nav/markers` | MarkerArray : passages candidats (orange), passage choisi (vert), flèche θ_cible (bleu), empreinte robot, impasses (rouge) | local_navigator |
| `/nav/path` | Path (trajectoire suivie) | local_navigator |
| `/nav/wheel_speeds_ideal` | [V_G, V_D] diagnostic | local_navigator |
| `/nav/enable` | Bool (démarrer / arrêter) | vous |
| `/wheel_speeds` | [v_G, v_D] mesurées | pico_bridge |

## 3. L'algorithme, pas à pas

**Étape 1 — Bandes de balayage.** Pour chaque direction φ entre −100° et +100° (pas de 2°), on considère une bande de largeur `W = largeur robot + 2 × marge` le long de φ. La **profondeur libre d(φ)** est la distance au premier point LiDAR dans la bande. Une ouverture plus étroite que W reste donc « bouchée » : **une ouverture vue par le LiDAR n'est pas automatiquement traversable**.

**Étape 2 — Passages.** Une direction est libre si `d(φ) ≥ min_passage_depth` **et** si la voiture peut l'atteindre en braquant (`2·sin φ / d ≤ κ_max = tan δ_max / L`). Les directions libres contiguës forment des plages, découpées aux creux de profondeur (un carrefour en T donne 2 passages). Pour chaque passage on calcule : direction, largeur disponible (la plus faible devant), profondeur, distance d'entrée, dégagement latéral, risque, fraction de zones **inconnues** sur la carte, proximité d'une impasse connue.

**Étape 3 — Score et θ_cible.**
`score = profondeur + dégagement latéral − écart à tout-droit − changement par rapport au θ précédent − zones inconnues − impasse connue`.
Le passage gardé change seulement si un autre le dépasse nettement (hystérésis). θ_cible = direction du passage + un **recentrage** basé sur des distances aux parois **robustes** (30e percentile sur 0,15–1,6 m, pas le point le plus proche), avec une zone morte de 4 cm, puis un filtre passe-bas et une limite de vitesse de rotation.
→ C'est ce qui fait que, dans le **zigzag**, les irrégularités ne font pas tourner la voiture (θ ≈ 0°, V_G ≈ V_D).

**Étape 4 — Commande (pure pursuit).** `κ = 2·sin θ / L_d` avec `L_d = 0,45 + 0,6·v` ; puis `v` limité par la distance libre, le virage (`v ≤ √(a_lat/|κ|)` et réduction proportionnelle à |θ|), le passage étroit (0,22 m/s, maintenu jusqu'à ce que l'arrière soit sorti), et l'accélération / décélération max. `ω = v·κ`.

**Étape 5 — Cas dégradés.**
- Obstacle frontal : un autre passage est choisi. Sans alternative : arrêt progressif (`STOPPING`).
- Impasse / blocage (pas de passage, ou voiture arrêtée malgré un passage) : arrêt, mémorisation de l'impasse, puis **demi-tour en plusieurs manœuvres** (une Ackermann ne tourne pas sur place) : marche arrière braquée, puis marche avant braquée de l'autre côté, jusqu'à 4 cycles. Si ce n'est pas possible : `STOPPED`.
- Perte du `/scan` > 0,5 s : arrêt. Perte de la pose SLAM : navigation locale seule à 60 % de la vitesse. Perte de `/cmd_vel` > 0,3 s : `pico_bridge` envoie speed = 0, et le Pico s'arrête de lui-même après 500 ms sans trame.

**Étape 6 — Usage de la carte SLAM.** Un obstacle connu sur `/map` raccourcit `d(φ)`. Les cases inconnues pénalisent le score : **l'inconnu n'est pas considéré comme libre**. Les impasses mémorisées pénalisent les directions qui y mènent, ce qui évite d'y retourner. L'exploration par frontières n'a pas été implémentée : pour un circuit en anneau elle n'apporte rien (on suit le seul couloir disponible).

## 4. Installation (sur le RPi5, et sur le PC pour RViz)

```bash
sudo apt update
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server python3-serial python3-numpy
# sur le PC uniquement :
sudo apt install ros-jazzy-rviz2
```

Copier le dossier `au516_nav` à la place de l'ancien dans le workspace (`rplidar_ros` doit être dans le même `src/`) :

```bash
cd ~/ros2_ws/src/AU516          # là où se trouvait l'ancien au516_nav
mv au516_nav au516_nav_old      # sauvegarde
cp -r <ce_dossier>/au516_nav .
cd ~/ros2_ws
colcon build --symlink-install --packages-select au516_nav
source install/setup.bash
```

## 5. Lancement

**A. Simulation (PC avec ROS 2, sans robot)**

```bash
ros2 launch au516_nav sim.launch.py scenario:=D_zigzag
# scénarios : A_straight B_left C_right D_zigzag E_narrow_ok E_narrow_refused
#             F_obstacle F_blocked G_dead_end T_junction ring
```

RViz s'ouvre : carte SLAM, scan, passages, θ_cible, trajectoire, murs simulés. `/sim/collision` passe à `true` en cas de choc.

**B. Robot réel, première fois : roues en l'air**

```bash
# terminal 1 (RPi5) : matériel + SLAM + navigation, navigation en pause
ros2 launch au516_nav bringup.launch.py
# terminal 2 (PC, même ROS_DOMAIN_ID) :
rviz2 -d ~/ros2_ws/install/au516_nav/share/au516_nav/rviz/au516_nav.rviz
# terminal 3 : démarrer / arrêter la navigation
ros2 topic pub --once /nav/enable std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /nav/enable std_msgs/msg/Bool "{data: false}"
```

Options : `slam:=false` (navigation locale seule), `navigate:=false` (cartographie manuelle), `lidar_port:=/dev/ttyUSB1`.

**C. Sauvegarder la carte**

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/piste_essai
```

**D. Diagnostic rapide**

```bash
ros2 run tf2_tools view_frames            # doit montrer map → odom → base_link → laser
ros2 topic hz /scan                       # ≈ 10 Hz
ros2 topic echo /nav/theta_target
ros2 topic echo /nav/state
```

## 6. Calibration (obligatoire avant de rouler)

| Paramètre | Fichier | Comment le vérifier |
|---|---|---|
| `wheelbase`, `width`, `front_overhang`, `rear_overhang` | `nav.yaml` (`vehicle.*`) **et** `pico_bridge.yaml` | mètre ruban. `base_link` = centre de l'essieu **arrière** |
| `laser_x`, `laser_yaw` | `nav.yaml` + arguments de `robot.launch.py` | objet à 50 cm devant : il doit apparaître droit devant dans RViz. Sinon `laser_yaw: 3.14159` |
| `steering_sign` | `pico_bridge.yaml` | `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.5}}"` roues en l'air : les roues doivent braquer à **gauche**. Sinon `-1.0` |
| `servo_center_us`, `servo_us_at_max_steer`, `max_steer` | `pico_bridge.yaml` | roues droites au centre ; mesurer l'angle à la butée |
| `encoder_left_sign`, `encoder_right_sign` | `pico_bridge.yaml` | en avançant, `ros2 topic echo /wheel_speeds` doit donner 2 valeurs **positives** |
| `wheel_diameter` | `pico_bridge.yaml` | rouler 2 m mesurés : `/odom` doit indiquer ≈ 2 m |
| `safety_margin`, `v_max` | `nav.yaml` | commencer avec `v_max: 0.3` |

## 7. Tests et résultats

```bash
cd ~/ros2_ws/src/AU516/au516_nav
python3 -m pytest test/test_scenarios.py test/test_map_memory.py -v   # sans ROS
python3 tools/plot_scenarios.py doc/                                   # figure + tableau (matplotlib)
```

| Test | Ce qui est vérifié |
|---|---|
| A — couloir droit | départ décalé (12 cm, 3°) corrigé ; abs(θ_cible) < 4°, braquage < 6° après stabilisation |
| B / C — virages | virage effectué sans collision ; V_G < V_D (gauche), V_G > V_D (droite) ; ralentissement |
| D — zigzag | écart latéral < 12 cm, θ_cible RMS < 4°, abs(V_G − V_D) < 0,05 m/s |
| E — passage 0,46 m | traversé à vitesse réduite |
| E — passage 0,30 m | refusé : la voiture n'entre pas, arrêt sans collision |
| F — obstacle | contourné ; si le couloir est bouché : arrêt sans collision |
| G — impasse | impasse mémorisée, demi-tour en manœuvres, repart dans l'autre sens |
| Circuit en anneau | ≥ 2 tours en 120 s sans collision |
| Carte / mémoire | une impasse connue fait choisir l'autre branche d'un T ; une zone inconnue est pénalisée ; un obstacle connu sur la carte bloque la direction |

Une étude de robustesse a aussi été faite (zigzag avec amplitude 0,15–0,4 m et période 0,5–1,8 m, départs décalés de ±25 cm et ±15°, 3 graines de bruit, passages de 0,38 à 0,60 m) : 0 échec.

Résultats détaillés : `doc/resultats_simulation.md`. Trajectoires : `doc/scenarios.png`. Temps de calcul mesuré : ≈ 18 ms par scan (720 rayons, avec carte) sur la machine de développement. Sur le RPi5, si c'est trop, mettre `beam_stride: 2`.

Note : les tests `test_flake8` / `test_pep257` d'ament échouaient déjà sur les fichiers d'origine (`scan_viewer.py`, `encoder_reader.py`). Les nouveaux fichiers respectent flake8 (99 colonnes).

## 8. Réglages utiles

| Symptôme | Paramètre (`nav.yaml`) |
|---|---|
| Oscille dans les lignes droites | baisser `centering_gain`, augmenter `centering_deadband` ou `w_stability` |
| Tourne trop tard | baisser `lookahead_cap` ; trop tôt : l'augmenter |
| Refuse des passages qu'il pourrait prendre | baisser `safety_margin` (avec prudence) |
| Trop lent dans les virages | augmenter `a_lat_max`, baisser `turn_slowdown` |
| S'arrête trop loin des obstacles | baisser `stop_distance` / `slow_distance` |
| Carte floue ou doublée | voir `Ressources/SLAM_2D.md` §10 étape 6 |

## 9. Fichiers créés et modifiés

**Créés**
- `au516_nav/nav_core/` : `params.py`, `geometry.py`, `navigator.py`, `control.py`, `mapping.py`, `sim2d.py`, `worlds.py` (cœur algorithmique sans ROS)
- `au516_nav/pico_bridge.py`, `au516_nav/local_navigator.py`, `au516_nav/sim_2d.py` (nœuds ROS 2)
- `config/slam_toolbox.yaml`, `config/nav.yaml`, `config/pico_bridge.yaml`
- `launch/robot.launch.py`, `slam.launch.py`, `navigation.launch.py`, `bringup.launch.py`, `sim.launch.py`
- `rviz/au516_nav.rviz`
- `test/test_scenarios.py`, `test/test_map_memory.py`, `tools/plot_scenarios.py`
- `doc/scenarios.png`, `doc/resultats_simulation.md`, ce `README_NAVIGATION.md`

**Modifiés**
- `setup.py` : nouveaux exécutables, installation des launch / config / rviz
- `package.xml` : dépendances (slam_toolbox, nav_msgs, tf2_ros, numpy, rplidar_ros, rviz2…)

**Inchangés** : `encoder_reader.py`, `scan_viewer.py`, `conversion_2D.py`, tout le code Pico (`Codes/`).

## 10. Limites connues

- Pas de Gazebo : `sim_2d` est un simulateur 2D léger (pas de glissement, pas d'inertie, LiDAR sans auto-occultation par la carrosserie).
- Le lacet de l'odométrie vient du modèle bicyclette avec le braquage **commandé**, pas mesuré. SLAM Toolbox corrige la dérive, mais une odométrie mal calibrée donnera une carte floue.
- Le demi-tour réussit dans un couloir de 1,6 m et échoue proprement (arrêt `STOPPED`, sans collision) dans un couloir de 1,2 m, avec la géométrie par défaut (rayon de braquage ≈ 0,52 m).
- Pour la course, la suite logique est le « tour 2+ » de `SLAM_2D.md` §11 : pure pursuit sur la trajectoire du tour 1 (le contrôleur actuel sert de couche réactive de sécurité).
