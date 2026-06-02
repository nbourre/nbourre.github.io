# Représentation Espace-État

## Qu'est-ce que l'état d'un système ?

L'**état** d'un système est l'ensemble minimal de variables qui permet de prédire son comportement futur, connaissant les règles de mouvement et les commandes appliquées.

!!! example "Exemple : le Ranger sur une ligne droite"
    Si le robot est à 30 cm d'un mur mais qu'on ignore sa vitesse, on ne peut pas prédire sa position dans 1 seconde. En revanche, si on connaît **position ET vitesse**, on peut prédire l'état suivant.
    
    L'état est donc : **x = [position, vitesse]ᵀ**

Le choix de l'état dépend de ce qu'on veut faire :

| Application | Variables d'état | Dimension |
|-------------|-----------------|-----------|
| Robot sur une ligne | position, vitesse | 2 |
| Robot en 2D | x, y, vx, vy | 4 |
| Robot avec orientation | x, y, θ, vx, vy, ω | 6 |
| Drone dans l'espace | x, y, z, roll, pitch, yaw + vitesses | 12 |

Le filtre de Kalman fonctionne avec **toutes ces dimensions** — seule la taille des matrices change.

## Le modèle discret

Un microcontrôleur travaille par pas de temps discrets (Δt). Le modèle d'état discret s'écrit :

$$\mathbf{x}_k = F \mathbf{x}_{k-1} + G \mathbf{u}_{k-1}$$

Et le modèle de mesure :

$$\mathbf{y}_k = H \mathbf{x}_k$$

Trois matrices définissent entièrement le système :

| Matrice | Nom | Rôle |
|---------|-----|------|
| **F** | Matrice de transition | Comment l'état évolue d'un pas au suivant |
| **G** | Matrice de commande | Comment les commandes (moteurs) affectent l'état |
| **H** | Matrice de mesure | Quelles parties de l'état les capteurs peuvent voir |

## Construction de F : le mouvement du Ranger

### Cas simple : position et vitesse constante

Le Ranger avance à vitesse constante. En Δt secondes :

```
position_suivante = position_actuelle + vitesse × Δt
vitesse_suivante  = vitesse_actuelle  (constante)
```

Sous forme matricielle :

$$\begin{bmatrix} x \\ v \end{bmatrix}_{k} = \underbrace{\begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix}}_{F} \begin{bmatrix} x \\ v \end{bmatrix}_{k-1}$$

Cette matrice F dit simplement : "avance la position de v×Δt, garde la vitesse identique."

### Cas avec commande moteur

Si on contrôle l'accélération *u* des moteurs :

$$F = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix}, \quad G = \begin{bmatrix} \frac{\Delta t^2}{2} \\ \Delta t \end{bmatrix}$$

L'accélération contribue à la fois à la position (terme en Δt²/2) et à la vitesse (terme en Δt).

## Construction de H : ce que voit chaque capteur

H sélectionne les parties de l'état qu'un capteur peut mesurer.

**Sonar** : mesure la position, pas la vitesse

$$H_\text{sonar} = \begin{bmatrix} 1 & 0 \end{bmatrix}$$

→ garde seulement la première composante (position)

**Encodeur** : mesure la vitesse, pas la position

$$H_\text{enc} = \begin{bmatrix} 0 & 1 \end{bmatrix}$$

→ garde seulement la deuxième composante (vitesse)

**Gyroscope + accéléromètre** : si l'état est [θ, ω] et que le gyro donne ω directement :

$$H_\text{gyro} = \begin{bmatrix} 0 & 1 \end{bmatrix}$$

## Exemple complet : orientation du Ranger

L'orientation du Ranger est suivie avec :

- **État** : θ (angle en degrés), ω (vitesse angulaire en deg/s)
- **Entrée** : différence de vitesse entre les roues (convertie en ω estimée)
- **Mesure** : gyroscope (donne ω directement)

```cpp
// Modèle d'état pour l'orientation
// État x = [θ, ω]ᵀ
//
// x_k = F * x_{k-1} + G * u_{k-1}
// y_k = H * x_k
//
// F = | 1  dt |   (θ avance de ω*dt, ω reste constant)
//     | 0  1  |
//
// G = | 0 |   (la commande moteur n'est pas directement modélisée ici)
//     | 1 |
//
// H = | 0  1 |   (le gyro mesure ω, pas θ directement)
```

## Visualiser le modèle en action

Ce sketch montre le modèle d'état en mode déterministe (sans bruit) — utile pour vérifier que F est correcte avant d'ajouter le filtre :

```cpp
#include <MeAuriga.h>

MeGyro gyro(0, 0x69);

// État : [angle_Z (deg), vitesse_angulaire (deg/s)]
float x[2] = {0.0, 0.0};  // État initial

float dt = 0.05;  // 50ms entre mises à jour

// Matrice de transition F = [[1, dt], [0, 1]]
// On l'applique manuellement ici

unsigned long prevTemps;

void setup() {
  Serial.begin(115200);
  gyro.begin();
  gyro.resetData();
  prevTemps = millis();
  Serial.println("AngleModele\tAngleGyro\tVitesse");
}

void loop() {
  unsigned long maintenant = millis();
  if (maintenant - prevTemps < (unsigned long)(dt * 1000)) return;
  float dtReel = (maintenant - prevTemps) / 1000.0;
  prevTemps = maintenant;

  gyro.update();

  // La "commande" ici est la vitesse angulaire mesurée par le gyro
  float omega_mesure = gyro.getAngleZ() - x[0];  // approximation
  // (En pratique, on lirait une vitesse angulaire dérivée)

  // Application du modèle F*x (prédiction pure, sans correction)
  float theta_pred = x[0] + x[1] * dtReel;
  float omega_pred = x[1];  // on suppose vitesse constante

  x[0] = theta_pred;
  x[1] = omega_pred;

  // Comparaison avec la mesure brute
  Serial.print(x[0], 2);
  Serial.print("\t");
  Serial.print(gyro.getAngleZ(), 2);
  Serial.print("\t");
  Serial.println(x[1], 2);
}
```

## Les matrices Q et R : les bruits

En réalité le modèle n'est pas parfait et les capteurs ont du bruit. On ajoute deux matrices de bruit :

$$\mathbf{x}_k = F \mathbf{x}_{k-1} + G \mathbf{u}_{k-1} + \mathbf{w}_k, \quad \mathbf{w}_k \sim \mathcal{N}(0, Q)$$
$$\mathbf{z}_k = H \mathbf{x}_k + \mathbf{v}_k, \quad \mathbf{v}_k \sim \mathcal{N}(0, R)$$

| Matrice | Nom | Équivalent 1D |
|---------|-----|--------------|
| **Q** | Bruit de procédé | σ²_enc (bruit encodeur) |
| **R** | Bruit de mesure | σ²_sonar (bruit sonar) |

Pour un état [θ, ω] avec un gyroscope comme seul capteur :

```
Q = | σ²_θ_process    0           |   (incertitude sur le modèle de mouvement)
    | 0               σ²_ω_process |

R = | σ²_gyro |   (bruit du gyroscope — mesuré au chapitre 3)
```

## Résumé

La représentation espace-état donne au filtre son modèle de mouvement :

| Composant | Définition |
|-----------|------------|
| **x** | Vecteur d'état (ce qu'on veut estimer) |
| **F** | Comment l'état évolue (physique du robot) |
| **G** | Comment les commandes affectent l'état |
| **H** | Ce que les capteurs peuvent voir |
| **Q** | Confiance dans le modèle F |
| **R** | Confiance dans les capteurs |

On a maintenant toutes les pièces. Dans le dernier chapitre, on assemble tout pour le filtre de Kalman multivarié complet.
