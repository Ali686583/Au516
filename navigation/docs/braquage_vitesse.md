# Cours : calculer le braquage et la vitesse pour prendre un virage

Ce cours applique la méthode de `Conception_ROS2_SLAM2D_LiDAR360_detaillee.md` :
- le **secteur avant** du LiDAR trouve la direction du passage, `θ_cible`, qui donne le **virage** ;
- le **secteur arrière** donne la **vitesse** `v`.

Sur notre voiture, le virage se fait en orientant les **roues avant** (angle `δ`, avec le servo). La vitesse est une **consigne `s`** envoyée aux deux moteurs par `set_speed`.

```text
scan ─┬─► AVANT ───► θ_cible ─► δ (braquage) ─► impulsion servo (µs)
      └─► ARRIÈRE ─► v (m/s) ─────────────────► consigne moteur s
```

Tous les exemples chiffrés ont été recalculés avec les valeurs réelles de la voiture.

---

## 0. Conventions et paramètres

**Signes** (convention ROS) : **gauche = positif**. Si `θ_cible > 0`, le passage est à gauche. Si `δ > 0`, on braque à gauche. Si `ω > 0`, la voiture tourne à gauche.

| Symbole | Nom | Valeur | D'où elle vient |
|---|---|---|---|
| `L` | empattement (essieu avant ↔ essieu arrière) | **0,165 m** | `odometry_node.py` |
| `δ_max` | braquage maximal | **20°** (1000 µs = 20° à gauche, 2000 µs = 20° à droite) | `Commands list`, `odometry_node.py` |
| `D` | diamètre des roues | **0,065 m** | `odometry_node.py` |
| `N` | impulsions d'encodeur par tour de roue | **≈ 1260** | `motor_driver.py` |
| `k_v` | conversion consigne → m/s | **≈ 0,0162 m/s par unité** | calculé au §5 |
| `e` | voie (distance entre roue gauche et roue droite) | 0,15 m | **exemple, à mesurer** |

👉 Conséquence directe : la consigne maximale `s = 50` correspond à `50 × 0,0162 ≈` **0,81 m/s**. C'est la vitesse maximale de la voiture.

---

## 1. Le modèle de la voiture (modèle bicyclette)

On regroupe les deux roues avant en une seule, et les deux roues arrière en une seule. Quand on braque d'un angle `δ`, la voiture tourne autour d'un centre situé sur le prolongement de l'essieu arrière :

```text
                 roue avant braquée de δ
                       ╲
                  ┌─────╲─┐
                  │       │ ↑
                  │  L    │ │ empattement
                  │       │ ↓
                  └───●───┘ ◄── centre de l'essieu arrière
                      │
                      │◄────────── R ──────────► ✚ centre du virage
```

Les trois formules de base :

```text
R = L / tan δ                  rayon du virage
κ = 1 / R = tan δ / L          courbure
ω = v / R = v · tan δ / L      vitesse de rotation de la voiture (rad/s)
```

C'est exactement le calcul que fait `odometry_node.py` (`delta_theta = distance * tan(steering_angle) / wheelbase`).

**Ce qui change par rapport à un robot à deux roues indépendantes :**
- **À l'arrêt, la voiture ne tourne pas** : si `v = 0`, alors `ω = 0`. Elle ne peut pas pivoter sur place.
- **Il y a un rayon minimal** : `R_min = L / tan δ_max = 0,165 / tan 20° ≈ 0,45 m`. Un virage plus serré est impossible.
- **La rotation maximale dépend de la vitesse** : `ω_max = v · tan δ_max / L`. À 0,6 m/s, cela donne ≈ 1,32 rad/s.

---

## 2. Trouver `θ_cible` (secteur avant)

Trois façons de choisir la direction du passage. On note `rᵢ` la distance mesurée à l'angle `αᵢ` :

```text
(a) Direction la plus dégagée      θ_cible = αᵢ  où  rᵢ est la plus grande
(b) Centre du plus grand « trou »  θ_cible = (α_début + α_fin) / 2
    (suite de points consécutifs avec rᵢ > d_seuil)
(c) Moyenne pondérée               θ_cible = Σ rᵢ·αᵢ / Σ rᵢ
```

- **(a)** est simple mais saute d'un point à l'autre.
- **(b)** correspond au « corridor traversable » du document de conception, et résiste bien aux zigzags.
- **(c)** donne un résultat lisse.

On garde aussi **`d_cible`**, la distance jusqu'au passage : la formule C ci-dessous en a besoin.

---

## 3. Du `θ_cible` au braquage `δ` : quatre formules

### Formule A : proportionnelle directe

```text
δ = K_δ · θ_cible
```

Exemple avec `K_δ = 0,8` et `θ_cible = 20°` : `δ = 16°`.

- ➕ Ultra simple.
- ➖ Ne tient compte ni de la vitesse, ni de la distance du passage.

### Formule B : la méthode du document de conception, en passant par `ω`

```text
ω = K_p · θ_cible
δ = atan( L · ω / v )
```

La seconde ligne vient de `ω = v · tan δ / L` (§1), qu'on retourne pour isoler `δ`.

Exemple avec `K_p = 1,5`, `θ_cible = 20°` (0,349 rad) et `v = 0,6 m/s` :
- `ω = 0,52 rad/s` ;
- `δ = atan(0,165 × 0,52 / 0,6) ≈ 8,2°`.

**À retenir :**
- **Pour un même `ω`, plus on va vite, moins on braque.** À grande vitesse, un petit coup de volant suffit à tourner beaucoup.
- **`K_p` règle la vitesse de correction.** Si le passage ne bouge pas, `θ_cible` diminue au rythme où la voiture tourne :

  ```text
  dθ_cible/dt = −ω = −K_p · θ_cible   ⟹   θ_cible(t) = θ₀ · e^(−t/τ)   avec τ = 1 / K_p
  ```

  Avec `K_p = 1,5`, on a `τ ≈ 0,67 s`, et 95 % de la correction est faite après `3τ ≈ 2 s`. C'est le virage « progressif et auto-correctif » du document de conception, mis en équation.
- ⚠️ **Division par zéro** quand `v → 0` : si `|v| < 0,05 m/s`, prendre `δ = 0`.

### Formule C : géométrique (« pure pursuit »)

On vise un point à la distance `d` dans la direction `θ_cible`. Un seul arc de cercle part de la voiture, tangent à sa direction, et passe par ce point :

```text
            point visé ●
                      ╱   ← arc de rayon R
                     ╱
                    ╱ d
          θ_cible  ╱
              [voiture]

R = d / (2 · sin θ_cible)
δ = atan( 2 · L · sin θ_cible / d )
```

Exemple avec `θ_cible = 20°` et `d = 0,8 m` : `R ≈ 1,17 m`, donc `δ ≈ 8,0°`.

- ➕ Physiquement exacte, et elle utilise une distance que le LiDAR mesure déjà.
- ➕ Un seul réglage : `d`. Un petit `d` donne une conduite nerveuse, un grand `d` une conduite douce. On peut prendre `d = min(d_cible, d_max)`.
- ⚠️ La formule suppose que les angles sont mesurés depuis l'essieu arrière, or le LiDAR est à l'avant, à `x_lidar` devant l'essieu. Pour corriger :

  ```text
  x = d·cos θ + x_lidar    y = d·sin θ    →    θ' = atan2(y, x)    d' = √(x² + y²)
  ```

Code testé : `../codes/pure_pursuit.py`.

### Formule D : amortir les oscillations

Ce n'est pas une formule de plus : c'est un complément à A, B ou C contre les zigzags.

```text
Filtre        θ_f(k) = β · θ_cible(k) + (1 − β) · θ_f(k−1)      β ∈ ]0 ; 1], ex. 0,5
Dérivée       δ = K_p · θ_f + K_d · (θ_f(k) − θ_f(k−1)) / Δt
Zone morte    si |θ_f| < 2°  →  δ = 0
```

- **Le filtre** lisse les sauts d'un scan à l'autre.
- **Le terme dérivé `K_d`** freine la correction quand elle va trop vite.
- **La zone morte** évite de réagir aux petites irrégularités des murs.

### Comparaison

| | A : directe | B : via ω | C : géométrique |
|---|---|---|---|
| Tient compte de la vitesse | ❌ | ✅ | ❌ (pas besoin) |
| Tient compte de la distance du passage | ❌ | ❌ | ✅ |
| Réglages | `K_δ` | `K_p` | `d` |
| Exemple (passage à 20°) | 16° | 8,2° | 8,0° |

**Conseil :** la **C** est la plus juste physiquement, et la **B** est la plus proche du document de conception. Dans les deux cas, ajouter la **D**.

### Garde-fous (toujours, quelle que soit la formule)

```text
1. Saturation          δ = clamp(δ, −δ_max, +δ_max)          (±20°)
2. Vitesse du servo    |δ(k) − δ(k−1)| ≤ δ̇_max · Δt
3. Vers le servo       impulsion = 1500 − 500 · δ / δ_max     puis clamp(1000, 2000)
```

Exemple : `δ = 8,0°` donne `1500 − 500 × 8,0 / 20 ≈ 1299 µs`, donc on braque à gauche. Le signe « − » vient de la convention de la voiture : 1000 µs = gauche.

---

## 4. Calculer la vitesse `v`

Dans la méthode du document de conception, `v = f(scan arrière)`. Voici plusieurs formes possibles pour `f`, et les limites à ajouter. Ici `v_max = 0,8 m/s`, juste sous la vitesse maximale de la voiture (0,81 m/s).

### V1 : constante (premiers essais)

```text
v = v₀
```

### V2 : selon la distance libre

```text
x = (d − d_stop) / (d_loin − d_stop)        puis bornée à [0 ; 1]
v = v_min + (v_max − v_min) · x
si d < d_stop : v = 0  (arrêt d'urgence)
```

`d` est la distance représentative du secteur choisi : le minimum ou la moyenne.

Exemple avec `d = 1,5 m`, `d_stop = 0,3`, `d_loin = 2,0`, `v_min = 0,2` et `v_max = 0,8` : `v ≈ 0,62 m/s`.

⚠️ Comme le LiDAR est monté à l'avant, le secteur arrière voit aussi le châssis. Ignorer les mesures très proches (par exemple `rᵢ < 0,15 m`).

### V3 : selon le braquage (on ralentit quand on tourne)

```text
v = v_max − (v_max − v_min) · |δ| / δ_max
```

Exemple avec `δ = 8,0°` : `v ≈ 0,56 m/s`.

### V4 : la limite physique du virage

Dans un virage, la voiture subit une accélération latérale `a_lat = v² / R`. Au-delà d'une certaine valeur `a_lat,max`, les pneus glissent :

```text
v² · tan|δ| / L ≤ a_lat,max     ⟹     v ≤ √( a_lat,max · L / tan|δ| )
```

Exemple avec `a_lat,max = 2 m/s²` et `δ = 8,0°` : `v ≤ 1,53 m/s`. Cette limite ne joue pas ici, la vitesse maximale de la voiture arrive avant.

**Pour mesurer `a_lat,max`** : tourner en rond à braquage fixe en augmentant la vitesse jusqu'à ce que la voiture glisse.

### Combiner, puis lisser

```text
v = min( v_V2 , v_V3 , v_V4 , v_max )
Rampe :  |v(k) − v(k−1)| ≤ a_max · Δt
```

La rampe évite les à-coups, le patinage, et le LiDAR qui pique du nez au freinage.

**Ordre des calculs :** calculer d'abord `δ`, puis `v`. Les formules A, C et D n'ont pas besoin de `v`. Avec la B, prendre le `v` du scan précédent, pour éviter que `δ` dépende de `v` qui dépend de `δ`.

---

## 5. La vitesse des roues

### 5.1 La consigne commune (ce que fait `set_speed`)

```text
s = round( v / k_v )        bornée à [−10 ; 50]
```

`k_v` se calcule avec le diamètre des roues `D` et le nombre d'impulsions par tour `N`. La consigne `s` est en impulsions par 10 ms :

```text
v = s × 100 / N × π·D       donc       k_v = 100 · π · D / N = 100 × π × 0,065 / 1260 ≈ 0,0162
```

- `× 100` : impulsions par seconde ;
- `/ N` : tours de roue par seconde ;
- `× π·D` : mètres par seconde.

**Vérifier `k_v` sur la piste** : rouler avec `set_speed(20)` pendant 3 s. On doit parcourir environ `20 × 0,0162 × 3 ≈` **0,97 m**. Sinon, recalculer `k_v = distance / (3 × 20)`.

Exemple : `v = 0,56 m/s` donne `s = 34`.

### 5.2 Les formules `V_G / V_D` restent vraies, mais pour les roues arrière

Dans un virage, la roue extérieure parcourt un plus grand cercle que la roue intérieure :

```text
V_G = v − ω · e/2         V_D = v + ω · e/2          avec ω = v · tan δ / L
```

C'est la formule du §13 du document de conception, avec `e` (la voie) à la place de son `L`.

**Le problème :** `set_speed` impose **la même vitesse** aux deux moteurs. Dans un virage, la roue intérieure est poussée trop vite et la roue extérieure retenue. Les pneus frottent, et la voiture **résiste au virage** (elle sous-vire). Au braquage maximal (`R_min ≈ 0,45 m`), l'écart nécessaire est d'environ **±16 %**.

**Option : le différentiel électronique.** Le driver accepte une consigne par moteur : `set_speed` dans `motor_driver.py` envoie déjà `[-speed, 0, speed, 0]`, c'est-à-dire le moteur M1 (gauche, monté à l'envers) et le moteur M3 (droite). On pourrait donc envoyer deux vitesses différentes :

```text
s_G = s · (1 − e · tan δ / (2L))        s_D = s · (1 + e · tan δ / (2L))
```

Exemple avec `δ = 8,0°` : le facteur vaut 0,064, donc `s_G ≈ 32` et `s_D ≈ 36`.

Cela demanderait d'ajouter une méthode à `motor_driver.py` et de modifier `main.py` sur le Pico, qui sont des fichiers de l'équipe : à discuter avec elle avant. Par exemple :

```python
def set_speeds(self, speed_left, speed_right):
    """ Different speeds for the left motor (M1) and the right motor (M3) """
    left = int(self.clamp(speed_left))
    right = int(self.clamp(speed_right))
    self.i2c.writeto_mem(CHIP_ADDR, MOTOR_FIXED_SPEED_MOTOR_ADDR, bytearray([-left, 0, right, 0]))
```

---

## 6. Les deux roues avant (géométrie d'Ackermann, pour comprendre)

Le servo ne commande qu'un seul angle `δ`. La tringlerie de direction braque un peu plus la roue intérieure, qui tourne sur un plus petit cercle :

```text
tan δ_int = L / (R − e/2)        tan δ_ext = L / (R + e/2)
```

Exemple avec `R = 1,17 m` : `δ_int ≈ 8,6°` et `δ_ext ≈ 7,5°`. Le `δ` du modèle bicyclette (8,0°) se situe entre les deux.

Il n'y a rien à programmer ici : c'est mécanique. Cela explique simplement pourquoi `δ` est une « moyenne ».

---

## 7. Exemple complet sur un scan

**Données :**
- passage à **20° à gauche**, à `d = 0,8 m` ;
- distance libre `d = 1,5 m` ;
- paramètres du §0.

| Étape | Calcul | Résultat |
|---|---|---|
| 1. Braquage (C) | `atan(2 × 0,165 × sin 20° / 0,8)` | `δ ≈ 8,0°` |
| 2. Servo | `1500 − 500 × 8,0 / 20` | **1299 µs** |
| 3. Vitesse V2 / V3 / V4 | 0,62 / 0,56 / 1,53 | `v = min = 0,56 m/s` |
| 4. Moteurs | `0,56 / 0,0162` | **`s = 34`** (ou 32 / 36 avec le différentiel) |
| 5. Envoi au Pico | | `/cmd_drive` = `[34, 1299]`, transmis par `serial_bridge` |

---

## 8. L'algorithme, à chaque scan (≈ 10 fois par seconde)

```python
theta, d_cible = analyse_avant(scan)                    # § 2
theta_f = beta * theta + (1 - beta) * theta_f           # filtre (D)
delta = atan(2 * L * sin(theta_f) / d_cible)            # formule C
delta = clamp(delta, -DELTA_MAX, DELTA_MAX)

d_libre = analyse_arriere(scan)                         # secteur arrière
v = min(v_distance(d_libre), v_braquage(delta), v_physique(delta), V_MAX)
v = rampe(v, v_prec, A_MAX * dt)

s = clamp(round(v / K_V), -10, 50)
impulsion = clamp(1500 - 500 * delta / DELTA_MAX, 1000, 2000)
publier("/cmd_drive", [s, impulsion])                   # lu par serial_bridge
```

---

## 9. Résumé des formules

| Quoi | Formule |
|---|---|
| Rayon du virage | `R = L / tan δ` |
| Rotation de la voiture | `ω = v · tan δ / L` |
| Braquage A | `δ = K_δ · θ_cible` |
| Braquage B | `ω = K_p · θ_cible`, puis `δ = atan(L·ω / v)` |
| Braquage C | `δ = atan(2L · sin θ_cible / d)` |
| Temps de correction (B) | `τ = 1 / K_p` |
| Servo | `impulsion = 1500 − 500 · δ / δ_max` |
| Vitesse selon la distance | `v = v_min + (v_max − v_min) · x` |
| Vitesse selon le braquage | `v = v_max − (v_max − v_min) · \|δ\| / δ_max` |
| Limite en virage | `v ≤ √(a_lat,max · L / tan\|δ\|)` |
| Consigne moteur | `s = round(v / k_v)`, avec `k_v = 100·π·D / N ≈ 0,0162` |
| Différentiel arrière | `s_G,D = s · (1 ∓ e · tan δ / 2L)` |

---

## 10. Exercices

1. `θ_cible = 10°` et `d = 1,0 m`. Que valent `δ` (formule C) et l'impulsion envoyée au servo ?
2. On braque à `δ = 15°` et `a_lat,max = 2 m/s²`. Quelle est la vitesse maximale dans ce virage ? La voiture peut-elle l'atteindre ?
3. Avec `K_p = 1,5`, combien de temps faut-il pour faire 95 % de la correction ?

**Réponses :**
1. `δ = atan(2 × 0,165 × sin 10° / 1,0) ≈ 3,3°`, donc une impulsion d'environ **1418 µs**.
2. `v ≤ √(2 × 0,165 / tan 15°) ≈` **1,11 m/s**. Non : la voiture plafonne à 0,81 m/s, donc ce virage passe à fond.
3. `3τ = 3 / 1,5 =` **2 s**.
