# 🤖 KinovaCraft — Planification de Trajectoire & Co-Simulation Kinova Gen3 6-DOF (MATLAB & ROS 2)

![MATLAB](https://img.shields.io/badge/MATLAB-R2021a%2B-orange.svg)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)
![Robot](https://img.shields.io/badge/Robot-Kinova%20Gen3%206--DOF-red.svg)
![Gripper](https://img.shields.io/badge/Gripper-Robotiq%202F--85-lightgrey.svg)
![Author](https://img.shields.io/badge/Author-Aziz-brightgreen.svg)

Projet complet de génération de formes paramétriques, résolution de cinématique inverse sécurisée, comparaison de 4 interpolations articulaires et co-simulation temps réel entre **MATLAB** et **ROS 2 / RViz2** pour le bras manipulateur **Kinova Gen3 à 6 DOF** équipé de la pince **Robotiq 2F-85**.

---

## 🌟 Fonctionnalités Clés

- **Modèle officiel Kinova Gen3 6-DOF** : Chargé via `importrobot('gen3_6dof.urdf')` (exactement 6 articulations mobiles contrôlées, assertion stricte `assert(numel(q) == 6)`).
- **Tool Center Point (TCP)** : Tracé continu émis directement au contact du **bout des mors** de la pince (`tcp_link`, Z = +12 cm de la bride).
- **Contrôle actif de la pince Robotiq 2F-85** : Fermeture au 1er point lors de l'attaque du dessin, maintien de la préhension pendant le tracé, et ouverture automatique au dernier point.
- **Formes paramétriques cartésiennes** :
  - `rose` (Courbe Rhodonée avec paramètre `n_petals`, période adaptée pour éviter les redoublements).
  - `fish` (Courbe du poisson paramétrique).
  - `square` (Carré fermé).
- **Cinématique Inverse (IK) robuste** : Encapsulation `try-catch`, rejet des singularités et tolérance d'erreur stricte ($< 10^{-3}\text{ m}$).
- **Benchmark de 4 méthodes d'interpolation** :
  1. Trapézoïdale de vitesse (`trapveltraj`)
  2. Polynôme cubique $C^1$ (`cubicpolytraj`)
  3. Polynôme quintique $C^2$ (`quinticpolytraj`)
  4. B-spline uniforme $C^2$ (`bsplinepolytraj`)
- **Indicateurs de performance automatisés** : Durée, temps de calcul, $\dot{q}_{\max}$, $\ddot{q}_{\max}$, jerk maximal ($\text{rad/s}^3$), échecs IK, validation des limites dynamiques et export CSV.
- **Co-simulation ROS 2 / RViz2 sans fil** : Streaming en direct via DDS (`/joint_states` et `/trajectory_path`) vers RViz2 sur machine hôte ou VM.
- **Zéro valeur en dur** : Tous les réglages proviennent exclusivement de `params.m`.

---

## 📁 Arborescence du Projet

```
KinovaCraft/
├── kinova_trajectory_matlab/           # Projet MATLAB principal
│   ├── main.m                         # Point d'entrée principal du pipeline
│   ├── params.m                       # Fichier de configuration unique (zéro valeur en dur)
│   ├── setup_urdf.sh                  # Script automatisé de préparation URDF / meshes
│   ├── README.md                      # Documentation du projet
│   ├── src/
│   │   ├── load_robot_model.m         # Import URDF, extraction dynamique EE (TCP) et limites
│   │   ├── generate_shape.m           # Générateur cartésien (rose, poisson, carré)
│   │   ├── solve_ik_for_shape.m       # Résolution IK sécurisée try-catch
│   │   ├── plan_trajectories.m        # Moteur des 4 méthodes de trajectoire
│   │   ├── evaluate_metrics.m         # Calcul des métriques, tableau console et export CSV
│   │   ├── plot_shape_3d.m            # Figure 3D de la forme et des waypoints
│   │   ├── plot_joint_profiles.m      # Profils q, q̇, q̈ avec lignes limites rouges
│   │   ├── plot_comparison_bar.m      # Bar chart comparatif multi-critères
│   │   ├── animate_robot.m            # Rendu vidéo MP4 rapide avec pince active
│   │   ├── publish_ros2_trajectory.m  # Nœud publisher ROS 2 en streaming synchronisé
│   │   └── urdf/                      # URDF 6-DOF adapté et meshes 3D
│   │       ├── gen3_6dof.urdf
│   │       └── meshes/
│   │           ├── arm/               # STL officiels du bras Gen3 6-DOF
│   │           └── gripper/           # DAE/STL de la pince Robotiq 2F-85
│   └── results/                       # Fichiers générés automatiquement
│       ├── forme_3d.png
│       ├── profils_articulaires.png
│       ├── comparaison_methodes.png
│       ├── metrics_comparison.csv
│       └── trajectoire.mp4
│
└── src/
    ├── ros2_kortex/                   # Définitions officielles Kinova ROS 2
    └── trajectory_planner_g6/         # Package ROS 2 pour la passerelle RViz2
        ├── launch/
        │   └── matlab_rviz.launch.py  # Lance robot_state_publisher + RViz2
        ├── rviz/
        │   └── trajectory_viz.rviz    # Configuration RViz2 (RobotModel + Path orange)
        ├── scripts/
        │   └── record_rviz.sh         # Script d'enregistrement vidéo RViz2
        ├── package.xml
        └── setup.py
```

---

## ⚙️ Prérequis

### Côté MATLAB (Machine hôte Windows / Mac / Linux)
- **MATLAB R2021a** ou ultérieur
- **Robotics System Toolbox** (obligatoire)
- **ROS Toolbox** (obligatoire pour la liaison en direct avec RViz2)

### Côté ROS 2 (Linux Ubuntu 22.04 / VM)
- **ROS 2 Humble**
- Packages ROS 2 :
  ```bash
  sudo apt update
  sudo apt install -y ros-humble-robot-state-publisher ros-humble-rviz2 ros-humble-xacro
  ```

---

## 🚀 Guide d'Exécution Pas-à-Pas

### 1. Préparation de l'URDF (une seule fois, sous Linux)
```bash
cd ~/KinovaCraft/kinova_trajectory_matlab
./setup_urdf.sh
```
Ce script copie les maillages STL/DAE, configure le repère TCP au bout des doigts et garantit strictement 6 DOF actifs.

### 2. Lancement du visualiseur RViz2 (dans le terminal Linux)
```bash
cd ~/KinovaCraft
source /opt/ros/humble/setup.bash
colcon build --packages-select trajectory_planner_g6 --symlink-install
source install/setup.bash

export ROS_DOMAIN_ID=30
ros2 launch trajectory_planner_g6 matlab_rviz.launch.py
```
> Le Kinova Gen3 6-DOF s'affiche dans RViz2 et attend les commandes de MATLAB.

### 3. Exécution de la simulation sous MATLAB
1. Ouvre MATLAB sur ta machine.
2. Définis le dossier de travail :
   ```matlab
   cd('chemin/vers/KinovaCraft/kinova_trajectory_matlab')
   ```
3. Vérifie les paramètres dans `params.m` :
   ```matlab
   p.shape.type     = 'rose';          % 'rose', 'fish', ou 'square'
   p.ros2.enable    = true;            % Activer la co-simulation
   p.ros2.domain_id = 30;              % Même Domain ID que la VM
   ```
4. Lance l'exécution :
   ```matlab
   main
   ```

---

## 📊 Résultats Produits

À chaque exécution, le dossier `results/` est alimenté avec :
| Fichier | Description |
|---|---|
| `forme_3d.png` | Forme paramétrique 3D, robot à la pose initiale et waypoints validés |
| `profils_articulaires.png` | Profils $q(t)$, $\dot{q}(t)$, $\ddot{q}(t)$ avec limites rouges pointillées |
| `comparaison_methodes.png` | Bar chart comparatif (vitesse max, accélération max, jerk, temps de calcul) |
| `metrics_comparison.csv` | Tableau synthétique exporté au format CSV |
| `trajectoire.mp4` | Vidéo d'animation fluide avec fermeture de pince et tracé au contact |

---

## 📚 Références et Sources

- **Kinova Robotics** : [ros2_kortex (GitHub)](https://github.com/Kinovarobotics/ros2_kortex) pour le modèle URDF officiel du Gen3 6-DOF.
- **MathWorks Robotics System Toolbox** :
  - `importrobot` : Importation d'arbres cinématiques rigides.
  - `inverseKinematics` : Solveur numérique d'IK avec contraintes de pondération.
  - `trapveltraj`, `cubicpolytraj`, `quinticpolytraj`, `bsplinepolytraj` : Outils de planification articulaire.
- **Robotiq** : [robotiq_description](https://github.com/PickNikRobotics/robotiq_description) pour les maillages DAE/STL de la pince 2F-85.

---

## 👤 Auteur

**Aziz** — Ingénierie Robotique & Systèmes Autonomes  
*Projet individuel réalisé avec passion.*
