# Visualisation RViz2 — Kinova Gen3 6-DOF

Ce package ROS 2 fournit la passerelle de visualisation 3D sous RViz2 pour le manipulateur **Kinova Gen3 6-DOF** avec pince **Robotiq 2F-85**, synchronisé en direct avec la simulation MATLAB.

---

## 1. Rôle du Package
- Lance `robot_state_publisher` avec le modèle URDF officiel du Kinova Gen3 6-DOF + pince Robotiq 2F-85.
- Lance `rviz2` configuré pour afficher le robot en 3D (`/joint_states`) et le tracé progressif de l'effecteur (`/trajectory_path`).
- N'exécute aucun calcul en dur : le robot écoute passivement les commandes diffusées par MATLAB via DDS.

---

## 2. Compilation

Depuis la racine de l'espace de travail `KinovaCraft` :

```bash
cd ~/KinovaCraft
colcon build --packages-select trajectory_planner_g6 --symlink-install
source install/setup.bash
```

---

## 3. Lancement de la Visualisation

```bash
export ROS_DOMAIN_ID=30
ros2 launch trajectory_planner_g6 matlab_rviz.launch.py
```

---

## 4. Architecture

| Fichier | Rôle |
|---|---|
| `launch/matlab_rviz.launch.py` | Lancement de `robot_state_publisher` et `rviz2` |
| `rviz/trajectory_viz.rviz` | Configuration RViz2 (RobotModel, Path orange, repère `base_link`) |
| `package.xml` | Déclaration des dépendances ROS 2 |
| `setup.py` | Configuration d'installation colcon |

---

## Auteur
- **Aziz** — Projet individuel
