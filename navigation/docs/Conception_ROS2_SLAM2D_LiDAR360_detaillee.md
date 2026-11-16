# Conception détaillée --- Navigation ROS 2 / SLAM 2D avec LiDAR 360°

## 1. Objectif

Le système étudié est un **robot terrestre à roues** utilisant **ROS
2**, un **LiDAR 2D à 360°** et un système de **SLAM 2D**.

L'idée de navigation repose sur une séparation fonctionnelle des
informations fournies par le même LiDAR :

-   **la partie AVANT du scan sert à déterminer le virage à effectuer et
    donc la différence de commande entre les roues ;**
-   **la partie ARRIÈRE du scan sert à faire varier la vitesse
    d'avancement ;**
-   les deux résultats sont ensuite combinés pour produire la commande
    finale des roues.

Le LiDAR n'est donc pas seulement utilisé pour détecter des obstacles.
Il sert à analyser la **forme du passage libre** autour du robot.

------------------------------------------------------------------------

# 2. Vue générale du robot

``` text
                           AVANT
                             ↑
                             │
                  zone utilisée pour
                  calculer le VIRAGE
                             │

                    \        │        /
                     \       │       /
                      \      │      /
                       \     │     /
                        \    │    /
                         \   │   /
                          \  │  /
                           \ │ /
                        ┌─────────┐
                        │ LiDAR   │
                        │  360°   │
                        │  ROBOT  │
                        └─────────┘
                         O       O
                       roue G  roue D
                           \   /
                            \ /
                             │
                  zone utilisée pour
                  calculer la VITESSE
                             │
                             ↓
                           ARRIÈRE
```

Le LiDAR fournit un scan complet sur 360° via :

``` text
/scan
```

Chaque mesure correspond à une distance (r_i) associée à un angle
(`\theta`{=tex}\_i) :

\[ (r_i,`\theta`{=tex}\_i) \]

------------------------------------------------------------------------

# 3. Découpage logique du LiDAR

Le LiDAR reste physiquement un seul capteur 360°.

C'est le logiciel ROS 2 qui sépare ses mesures en plusieurs secteurs.

Conceptuellement :

``` text
                         0° / AVANT
                              ↑
                              │
                 ┌────────────┴────────────┐
                 │                         │
                 │     SECTEUR AVANT       │
                 │                         │
                 │   calcul du VIRAGE      │
                 │                         │
          GAUCHE │        [ROBOT]          │ DROITE
                 │                         │
                 │   calcul de VITESSE     │
                 │                         │
                 │     SECTEUR ARRIÈRE     │
                 └────────────┬────────────┘
                              │
                              ↓
                         180° / ARRIÈRE
```

Les limites angulaires exactes des secteurs pourront être ajustées
expérimentalement.

------------------------------------------------------------------------

# 4. Rôle de la partie AVANT : déterminer le virage

La partie avant du LiDAR regarde la géométrie située devant le robot.

Son objectif est de répondre à la question :

> **Dans quelle direction le passage réellement traversable
> continue-t-il ?**

Le système cherche donc un axe de passage libre :

\[ `\theta`{=tex}\_{cible} \]

Si cet axe est pratiquement aligné avec le robot :

\[ `\theta`{=tex}\_{cible}`\approx0`{=tex} \]

le robot continue droit.

Si l'axe se déplace vers la gauche, le robot commence à tourner à
gauche.

S'il se déplace vers la droite, il commence à tourner à droite.

------------------------------------------------------------------------

# 5. Couloir droit

Dans un couloir droit :

``` text
┌─────────────────────────────────────────────────────┐
│                                                     │
│                        ↑                            │
│                        │                            │
│                        │                            │
│                     [ ROBOT ]                       │
│                     O       O                       │
│                                                     │
└─────────────────────────────────────────────────────┘
```

Le passage libre est centré devant le robot :

\[ `\theta`{=tex}\_{cible}`\approx0`{=tex}° \]

Il n'est donc pas nécessaire de créer un virage.

Les deux roues reçoivent approximativement la même vitesse :

\[ V_G`\approx `{=tex}V_D \]

``` text
             ↑
             │
          [ROBOT]

        O         O
       VG   ≈    VD

             ↑
        TOUT DROIT
```

------------------------------------------------------------------------

# 6. Apparition progressive d'un virage à gauche

Supposons maintenant que le couloir commence à partir vers la gauche.

Le LiDAR avant commence à voir de plus en plus d'espace disponible dans
cette direction.

``` text
                     passage
                       ↖
                      ↖
                     ↖
                    /
                   /
        ┌─────────
        │
        │       [ ROBOT ]
        │       O       O
        │
────────┘
```

Au début, le décalage est faible :

``` text
θcible = 5°
```

Le robot effectue une petite correction.

Puis :

``` text
θcible = 15°
```

La correction augmente.

Puis :

``` text
θcible = 30°
```

Le virage devient plus marqué.

Le principe recherché est donc :

``` text
petit angle
    ↓
petite différence entre les roues
    ↓
virage doux


grand angle
    ↓
grande différence entre les roues
    ↓
virage plus important
```

------------------------------------------------------------------------

# 7. Apparition progressive d'un virage à droite

Le fonctionnement est symétrique.

``` text
                         passage
                              ↗
                               ↗
                                ↗
                                  \
                                   \
                                  [ ROBOT ]
                                  O       O
```

Si le passage libre se décale vers la droite :

\[ `\theta`{=tex}\_{cible}\<0 \]

selon la convention angulaire choisie.

La roue gauche doit alors avancer plus vite que la roue droite :

\[ V_G\>V_D \]

------------------------------------------------------------------------

# 8. Principe fondamental des deux roues

Pour un robot à deux roues motrices différentielles :

### Tout droit

\[ V_G=V_D \]

``` text
              ↑
           [ROBOT]
          O       O
         0.5     0.5
              ↑
```

### Virage à gauche

\[ V_G\<V_D \]

``` text
              ↖
           [ROBOT]
          O       O
        lent    rapide
              ↖
```

### Virage à droite

\[ V_G\>V_D \]

``` text
              ↗
           [ROBOT]
          O       O
       rapide    lent
              ↗
```

Plus l'écart entre (V_G) et (V_D) est important, plus le robot change
rapidement d'orientation.

------------------------------------------------------------------------

# 9. Calcul du virage demandé par l'avant

On définit l'erreur angulaire :

\[ e\_`\theta`{=tex}=`\theta`{=tex}*{cible}-`\theta`{=tex}*{robot} \]

Dans le repère local du robot, son axe avant peut être considéré comme :

\[ `\theta`{=tex}\_{robot}=0 \]

Donc :

\[ e\_`\theta`{=tex}=`\theta`{=tex}\_{cible} \]

Exemple :

``` text
                    axe du passage
                         ↖
                        / 
                       / 30°
                      /
                     ↑
                     │ axe actuel
                  [ROBOT]
```

On a :

\[ e\_`\theta=30`{=tex}° \]

Conversion en radians :

\[ `\boxed{
e_{\theta,rad}=e_{\theta,deg}\frac{\pi}{180}
}`{=tex} \]

Ainsi :

\[ 30°`\approx0`{=tex},524 rad \]

------------------------------------------------------------------------

# 10. Transformer l'erreur en vitesse angulaire

Une première loi simple est :

\[ `\boxed{
\omega=K_p e_\theta
}`{=tex} \]

où :

-   (e\_`\theta`{=tex}) = erreur de direction ;
-   (K_p) = coefficient de réglage ;
-   (`\omega`{=tex}) = vitesse angulaire demandée au robot.

Ainsi :

``` text
angle faible
    ↓
ω faible
    ↓
petit virage


angle important
    ↓
ω important
    ↓
virage plus fort
```

Cette commande doit ensuite être limitée :

\[
-`\omega`{=tex}*{max}`\leq`{=tex}`\omega`{=tex}`\leq`{=tex}`\omega`{=tex}*{max}
\]

afin d'éviter une rotation physiquement trop rapide.

------------------------------------------------------------------------

# 11. Rôle de la partie ARRIÈRE : modifier la vitesse

Le point spécifique du concept est que **la partie arrière du LiDAR
possède une fonction différente de la partie avant**.

Elle ne choisit pas directement l'angle du virage.

Elle intervient sur la **vitesse générale d'avancement** du robot.

On sépare donc explicitement :

``` text
                LiDAR 360°
                     │
          ┌──────────┴──────────┐
          │                     │
          ▼                     ▼

      AVANT                  ARRIÈRE
          │                     │
          ▼                     ▼
       VIRAGE                 VITESSE
          │                     │
          ▼                     ▼
          ω                     v
```

La règle exacte permettant à l'arrière de modifier (v) devra être
définie selon ce que les mesures arrière doivent représenter dans le
comportement final.

Mathématiquement, on peut pour l'instant écrire :

\[ `\boxed{
v=f(\text{scan arrière})
}`{=tex} \]

avec une limitation physique :

\[ v\_{min}`\leq `{=tex}v`\leq `{=tex}v\_{max} \]

Le résultat de la zone arrière est donc une consigne de vitesse :

\[ `\boxed{v}`{=tex} \]

------------------------------------------------------------------------

# 12. Le point essentiel : AVANT et ARRIÈRE ne commandent pas séparément les moteurs

L'avant calcule une **commande de rotation** :

\[ `\omega`{=tex} \]

L'arrière calcule une **commande de vitesse** :

\[ v \]

Ensuite seulement, ces deux informations sont fusionnées.

``` text
       INFORMATION AVANT          INFORMATION ARRIÈRE
              │                           │
              ▼                           ▼
      direction du passage         vitesse souhaitée
              │                           │
              ▼                           ▼
              ω                           v
              │                           │
              └─────────────┬─────────────┘
                            │
                            ▼
                    CALCUL DES ROUES
```

------------------------------------------------------------------------

# 13. Formules finales des roues

On note :

-   \(L\) : distance entre les deux roues ;
-   \(v\) : vitesse linéaire demandée ;
-   (`\omega`{=tex}) : vitesse angulaire demandée.

Les vitesses linéaires des roues sont :

\[ `\boxed{
V_G=v-\frac{\omega L}{2}
}`{=tex} \]

\[ `\boxed{
V_D=v+\frac{\omega L}{2}
}`{=tex} \]

Ces équations réalisent exactement la fusion entre :

-   **la vitesse**, calculée par la logique arrière ;
-   **le virage**, calculé par la logique avant.

------------------------------------------------------------------------

# 14. Exemple complet

Supposons :

``` text
LiDAR AVANT :
passage détecté à 30° vers la gauche

LiDAR ARRIÈRE :
logique de vitesse → v = 0,40 m/s
```

L'avant donne :

\[ e\_`\theta=30`{=tex}°=0,524rad \]

Si :

\[ K_p=0,67 \]

alors approximativement :

\[ `\omega=0`{=tex},35rad/s \]

Supposons également :

\[ L=0,40m \]

Alors :

\[ V_G = 0,40-`\frac{0,35\times0,40}{2}`{=tex} \]

\[ `\boxed{V_G=0,33m/s}`{=tex} \]

et :

\[ V_D = 0,40+`\frac{0,35\times0,40}{2}`{=tex} \]

\[ `\boxed{V_D=0,47m/s}`{=tex} \]

Résultat :

``` text
                  PASSAGE
                     ↖
                    / 30°
                   /
                  /
               [ ROBOT ]
               O       O
               │       │
               │       │
          0,33 m/s   0,47 m/s
             VG         VD

                  ↖
            VIRAGE GAUCHE
```

La vitesse moyenne reste :

\[ `\frac{0,33+0,47}{2}`{=tex}=0,40m/s \]

La différence entre les deux roues crée le virage.

------------------------------------------------------------------------

# 15. Correction en boucle

Le robot ne doit pas recevoir une commande de 30° puis exécuter
aveuglément tout le virage.

Le LiDAR doit recalculer continuellement la direction du passage.

``` text
SCAN 1

passage = 30° gauche
        ↓
ω important
        ↓
forte différence VG / VD
        ↓
robot tourne
        ↓

SCAN 2

passage = 18° gauche
        ↓
ω diminue
        ↓
différence VG / VD diminue
        ↓
robot tourne encore
        ↓

SCAN 3

passage = 6° gauche
        ↓
petite correction
        ↓

SCAN 4

passage ≈ 0°
        ↓
ω ≈ 0
        ↓
VG ≈ VD
        ↓
TOUT DROIT
```

Cette boucle permet un virage progressif et auto-correctif.

------------------------------------------------------------------------

# 16. Gestion des zigzags

Le robot ne doit pas chercher à rester exactement au centre entre les
deux murs à chaque instant.

Sinon :

``` text
mur gauche change
       ↓
robot corrige à droite
       ↓
mur droit change
       ↓
robot corrige à gauche
       ↓
oscillations
```

Le système doit chercher un **corridor traversable**.

Exemple :

``` text
MUR GAUCHE

────────┐       ┌────────┐       ┌────────
        │       │        │       │
        └───────┘        └───────┘


                    ↑
                    │
                    │
                    │
                 [ROBOT]
                 O     O
                    │
                    │
                    ↑


        ┌───────┐        ┌───────┐
────────┘       └────────┘       └────────

MUR DROIT
```

Même si les murs zigzaguent :

``` text
       passage central libre
                 │
                 ▼
          θcible ≈ 0°
                 │
                 ▼
              ω ≈ 0
                 │
                 ▼
             VG ≈ VD
                 │
                 ▼
            TOUT DROIT
```

Ainsi, les irrégularités locales des murs ne provoquent pas
automatiquement un zigzag du robot.

------------------------------------------------------------------------

# 17. Schéma logique détaillé

``` text
                           ┌────────────────┐
                           │  LiDAR 360°    │
                           │     /scan      │
                           └───────┬────────┘
                                   │
                    séparation logicielle du scan
                                   │
                  ┌────────────────┴────────────────┐
                  │                                 │
                  ▼                                 ▼

         ┌─────────────────┐              ┌─────────────────┐
         │  SECTEUR AVANT  │              │ SECTEUR ARRIÈRE │
         └────────┬────────┘              └────────┬────────┘
                  │                                │
                  ▼                                ▼
        géométrie du passage              information utilisée
                  │                       pour régler la vitesse
                  ▼                                │
       recherche corridor libre                    ▼
                  │                                v
                  ▼                                │
             θ cible                               │
                  │                                │
                  ▼                                │
       eθ = θcible - θrobot                        │
                  │                                │
                  ▼                                │
             ω = Kp·eθ                             │
                  │                                │
                  └───────────────┬────────────────┘
                                  │
                                  ▼
                       ┌─────────────────────┐
                       │ FUSION v + ω        │
                       └──────────┬──────────┘
                                  │
                    ┌─────────────┴─────────────┐
                    │                           │
                    ▼                           ▼

          VG = v - (ωL/2)             VD = v + (ωL/2)

                    │                           │
                    ▼                           ▼
               ROUE GAUCHE                ROUE DROITE
                    │                           │
                    └─────────────┬─────────────┘
                                  │
                                  ▼
                              ROBOT
                                  │
                                  ▼
                         nouveau scan LiDAR
                                  │
                                  └───────────────► boucle
```

------------------------------------------------------------------------

# 18. Schéma physique du principe

``` text
                                AVANT

                                  ↑
                                  │
                           passage détecté
                                  ↖
                                   \
                                    \ θ
                                     \
                                  [ LiDAR ]
                              ┌──────[360°]──────┐
                              │      ROBOT       │
                              └──────────────────┘

                            O                    O
                         ROUE G              ROUE D
                            │                    │
                            │                    │
                 v - (ωL/2) │                    │ v + (ωL/2)
                            │                    │
                            └─────────┬──────────┘
                                      │
                                      │
                       ARRIÈRE → ajuste v
                                      │
                                      ↓

                                ARRIÈRE
```

------------------------------------------------------------------------

# 19. Relation avec ROS 2

Le fonctionnement logiciel envisagé est :

``` text
                        /scan
                          │
                          ▼
                 NODE TRAITEMENT LIDAR
                          │
             ┌────────────┴────────────┐
             │                         │
             ▼                         ▼
       analyse avant              analyse arrière
             │                         │
             ▼                         ▼
         θ cible                       v
             │                         │
             ▼                         │
             ω                         │
             └────────────┬────────────┘
                          │
                          ▼
                   calcul v + ω
                          │
                          ▼
                       /cmd_vel
                          │
                ┌─────────┴─────────┐
                │                   │
          linear.x = v       angular.z = ω
                │                   │
                └─────────┬─────────┘
                          ▼
                  base controller
                          │
                          ▼
                    roues du robot
```

Dans ROS 2, `/cmd_vel` exprime généralement directement :

``` text
linear.x  → vitesse d'avancement v
angular.z → vitesse de rotation ω
```

Le contrôleur différentiel peut ensuite convertir ces deux valeurs en
commandes des roues gauche et droite.

------------------------------------------------------------------------

# 20. Relation avec le SLAM 2D

Le même LiDAR peut également alimenter le SLAM 2D.

``` text
                     LiDAR 360°
                         │
                       /scan
                         │
             ┌───────────┴───────────┐
             │                       │
             ▼                       ▼
          SLAM 2D              NAVIGATION LOCALE
             │                       │
             ▼                       ├── AVANT → ω
            /map                     │
                                     └── ARRIÈRE → v
```

Les deux fonctions sont différentes :

### SLAM 2D

Il sert principalement à :

-   construire la carte ;
-   localiser le robot ;
-   conserver une représentation globale de l'environnement.

### Traitement local du LiDAR

Il sert à :

-   analyser immédiatement le passage ;
-   déterminer le virage ;
-   adapter la vitesse ;
-   éviter de suivre inutilement les zigzags des murs.

------------------------------------------------------------------------

# 21. Résumé mathématique

## Information avant

Le LiDAR avant fournit :

\[ `\theta`{=tex}\_{cible} \]

Puis :

\[ `\boxed{
e_\theta=\theta_{cible}-\theta_{robot}
}`{=tex} \]

Puis :

\[ `\boxed{
\omega=K_p e_\theta
}`{=tex} \]

Donc :

``` text
AVANT → θ → erreur → ω → VIRAGE
```

## Information arrière

La partie arrière fournit une information permettant de calculer :

\[ `\boxed{
v=f(\text{scan arrière})
}`{=tex} \]

Donc :

``` text
ARRIÈRE → calcul → v → VITESSE
```

## Fusion

Enfin :

\[ `\boxed{
V_G=v-\frac{\omega L}{2}
}`{=tex} \]

\[ `\boxed{
V_D=v+\frac{\omega L}{2}
}`{=tex} \]

Donc :

``` text
          AVANT
            │
            ▼
            ω
            │
            │
            ├─────────┐
            │         │
            │         ▼
            │      FUSION
            │         ▲
            │         │
            └─────────┤
                      │
                      v
                      ▲
                      │
                   ARRIÈRE

                      ↓

              VG             VD
```

------------------------------------------------------------------------

# 22. Idée centrale du système

Le comportement recherché peut finalement être résumé ainsi :

``` text
1. Le LiDAR 360° scanne l'environnement.

2. La zone AVANT cherche l'axe du passage libre.

3. Plus cet axe part à gauche ou à droite,
   plus le système augmente progressivement le virage.

4. Le virage est représenté par ω.

5. La zone ARRIÈRE est utilisée séparément
   pour faire évoluer la vitesse d'avancement v.

6. Les deux informations sont fusionnées :

        AVANT  → ω
                    \
                     → calcul des roues → VG / VD
                    /
        ARRIÈRE → v

7. Si un passage central reste libre malgré des murs
   en zigzag, le robot privilégie ce passage et continue
   autant que possible tout droit.

8. À chaque nouveau scan, les valeurs sont recalculées.
   Le robot corrige donc continuellement sa trajectoire.
```

## Formules essentielles

\[ `\boxed{
\theta_{rad}=\theta_{deg}\frac{\pi}{180}
}`{=tex} \]

\[ `\boxed{
\omega=K_p e_\theta
}`{=tex} \]

\[ `\boxed{
V_G=v-\frac{\omega L}{2}
}`{=tex} \]

\[ `\boxed{
V_D=v+\frac{\omega L}{2}
}`{=tex} \]

Ce document décrit l'architecture conceptuelle actuelle. La prochaine
étape de conception consiste à définir précisément **la loi de vitesse
basée sur le secteur arrière du LiDAR**, puis les secteurs angulaires,
seuils de sécurité, filtrages et saturations utilisés par l'algorithme
ROS 2.
