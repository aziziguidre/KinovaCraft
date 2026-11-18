#  KinovaCraft — Trajectory Planning & Co-Simulation for Kinova Gen3 6-DOF (MATLAB & ROS 2)

![MATLAB](https://img.shields.io/badge/MATLAB-R2021a%2B-orange.svg)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)
![Robot](https://img.shields.io/badge/Robot-Kinova%20Gen3%206--DOF-red.svg)
![Gripper](https://img.shields.io/badge/Gripper-Robotiq%202F--85-lightgrey.svg)
![Team](https://img.shields.io/badge/Team-Groupe%20G6-brightgreen.svg)
![Institution](https://img.shields.io/badge/Institution-ENSA%20Fès%20(USMBA)-blueviolet.svg)

Comprehensive project for parametric shape generation, safe inverse kinematics resolution, benchmark of 4 joint-space interpolation methods, and real-time co-simulation between **MATLAB** and **ROS 2 / RViz2** for the **Kinova Gen3 6-DOF** manipulator equipped with the **Robotiq 2F-85** gripper.

---

##  Preview & Visual Results

### 1. Kinova Gen3 Animated Simulation (Trajectory at Tool Center Point)
The Robotiq gripper closes at the very first point, firmly holds the tool throughout the entire trajectory tracing, and the continuous orange path is drawn directly from the tip of the closed fingers (TCP).

<p align="center">
  <img src="kinova_trajectory_matlab/results/trajectoire_preview.gif" alt="Kinova Gen3 Animated Simulation" width="720"/>
</p>

---

### 2. 3D Parametric Shape & Target Waypoints
3D visualization showing the manipulator initial pose, the closed trajectory path, and the distribution of waypoints validated through inverse kinematics ($< 10^{-3}\text{ m}$).

<p align="center">
  <img src="kinova_trajectory_matlab/results/forme_3d.png" alt="3D Shape and Waypoints" width="700"/>
</p>

---

### 3. Complete Joint Profiles — Quintic Polynomial Method
Time evolution of joint positions $q_i(t)$, velocities $\dot{q}_i(t)$, and accelerations $\ddot{q}_i(t)$ for the 6 joints with physical limit boundaries indicated by red dashed lines.

<p align="center">
  <img src="kinova_trajectory_matlab/results/profils_articulaires.png" alt="Joint Profiles" width="700"/>
</p>

---

### 4. Benchmark Comparison of the 4 Methods
Quantitative comparison across 4 key metrics: **Maximum Velocity**, **Maximum Acceleration**, **Maximum Jerk** (smoothness/chatter), and **Algorithmic Computation Time**.

<p align="center">
  <img src="kinova_trajectory_matlab/results/comparaison_methodes.png" alt="Method Comparison" width="700"/>
</p>

#### Summary Metrics Table (`metrics_comparison.csv` — Nominal Run $T = 20.0\text{ s}$):
| Interpolation Method | Total Duration ($s$) | Computation Time ($ms$) | Max Velocity ($\text{rad/s}$) | Max Acceleration ($\text{rad/s}^2$) | Max Jerk ($\text{rad/s}^3$) | IK Failures | Limits OK | Industrial Verdict |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **`trapveltraj`** (Trapezoidal) | 20.0 | 2.26 | 0.455 | 3.34 | 133.03 | 0 | **YES** ✅ | Compliant (step accelerations) |
| **`cubicpolytraj`** (Cubic $C^1$) | 20.0 | 1.89 | 0.455 | 4.44 | 155.75 | 0 | **YES** ✅ | Compliant ($C^1$ velocity continuous) |
| **`quinticpolytraj`** (Quintic $C^2$) | 20.0 | 2.33 | 0.568 | 4.23 | 84.56 | 0 | **YES** ✅ | **Recommended ($C^2$ smooth jerk)** |
| **`bsplinepolytraj`** (B-spline $C^2$) | 20.0 | **0.65** | **0.367** | **0.59** | **10.65** | 0 | **YES** ✅ | **Optimal (Lowest jerk & computation)** |

> *Physical Scaling Law ($a \propto 1/T^2$):* Over a high-speed duration of $T = 5.0\,\text{s}$, peak accelerations reached $63.6\,\text{rad/s}^2$ and jerks $2260\,\text{rad/s}^3$ (exceeding joint limits). Extending the duration to $T = 20.0\,\text{s}$ ($\sigma = 4$) divides velocities by $4$, accelerations by $16$ ($< 5.0\,\text{rad/s}^2$), and jerks by $64$, ensuring 100% compliance with Harmonic Drive actuators.

---

##  Key Features

- **Official Kinova Gen3 6-DOF Model**: Loaded via `importrobot('gen3_6dof.urdf')` (strictly 6 active controlled joints, enforced by `assert(numel(q) == 6)`).
- **Tool Center Point (TCP)**: Continuous trail drawn directly at the **tip of the gripper fingers** (`tcp_link`, Z = +12 cm from the wrist flange).
- **Active Robotiq 2F-85 Gripper Control**: Automatic closing at the first point when starting the shape, holding during trajectory tracing, and automatic opening at the final point.
- **Cartesian Parametric Shapes**:
  - `rose` (Rhodonea rose curve with configurable `n_petals`, adapted period to prevent redundant loops).
  - `fish` (Parametric fish curve).
  - `square` (Closed square path).
- **Robust Inverse Kinematics (IK)**: Encapsulated in `try-catch` blocks, singularity rejection, and strict error tolerance ($< 10^{-3}\text{ m}$).
- **Benchmark of 4 Interpolation Methods**:
  1. Trapezoidal velocity profile (`trapveltraj`)
  2. Cubic piecewise polynomial $C^1$ (`cubicpolytraj`)
  3. Quintic piecewise polynomial $C^2$ (`quinticpolytraj`)
  4. Uniform B-spline $C^2$ (`bsplinepolytraj`)
- **Automated Performance Metrics**: Duration, computation time, $\dot{q}_{\max}$, $\ddot{q}_{\max}$, maximum jerk ($\text{rad/s}^3$), IK failures, physical limits compliance boolean, and CSV export.
- **Wireless ROS 2 / RViz2 Co-Simulation**: Real-time DDS streaming (`/joint_states` and `/trajectory_path`) to RViz2 running on a host machine or VM.
- **Zero Hard-Coded Values**: Every parameter is loaded exclusively from `params.m`.

---

##  Repository Structure

```
KinovaCraft/
├── README.md                          # Main project documentation with visual results
├── .gitignore                         # Ignored files (ROS 2 build artifacts, MATLAB temp files)
├── kinova_trajectory_matlab/           # Main MATLAB project
│   ├── main.m                         # Main entry point pipeline
│   ├── params.m                       # Central configuration file (zero hard-coded values)
│   ├── setup_urdf.sh                  # Automated URDF / mesh setup script
│   ├── README.md                      # Inner MATLAB package documentation
│   ├── src/
│   │   ├── load_robot_model.m         # Import URDF, dynamic EE (TCP) extraction & limits
│   │   ├── generate_shape.m           # Cartesian shape generator (rose, fish, square)
│   │   ├── solve_ik_for_shape.m       # Safe IK solver with try-catch
│   │   ├── plan_trajectories.m        # Core engine for the 4 trajectory methods
│   │   ├── evaluate_metrics.m         # Metrics evaluation, console table & CSV export
│   │   ├── plot_shape_3d.m            # 3D plot of shape and waypoints
│   │   ├── plot_joint_profiles.m      # q, q̇, q̈ joint profiles with red limit lines
│   │   ├── plot_comparison_bar.m      # Multi-criteria comparative bar chart
│   │   ├── animate_robot.m            # Fast MP4 video renderer with active gripper
│   │   ├── publish_ros2_trajectory.m  # Synchronized streaming ROS 2 publisher node
│   │   └── urdf/                      # Adapted 6-DOF URDF and 3D meshes
│   │       ├── gen3_6dof.urdf
│   │       └── meshes/
│   │           ├── arm/               # Official Gen3 6-DOF arm STL files
│   │           └── gripper/           # Robotiq 2F-85 gripper DAE/STL files
│   └── results/                       # Automatically generated results
│       ├── forme_3d.png
│       ├── profils_articulaires.png
│       ├── comparaison_methodes.png
│       ├── metrics_comparison.csv
│       ├── trajectoire.mp4
│       └── trajectoire_preview.gif
│
└── src/
    ├── ros2_kortex/                   # Official Kinova ROS 2 definitions (kortex_description)
    └── trajectory_planner_g6/         # ROS 2 bridge package for RViz2
        ├── launch/
        │   └── matlab_rviz.launch.py  # Launches robot_state_publisher + RViz2
        ├── rviz/
        │   └── trajectory_viz.rviz    # RViz2 configuration (RobotModel + orange Path)
        ├── scripts/
        │   └── record_rviz.sh         # RViz2 video recording script
        ├── package.xml
        └── setup.py
```

---

##  Prerequisites

### MATLAB Side (Host Machine Windows / Mac / Linux)
- **MATLAB R2021a** or later
- **Robotics System Toolbox** (required)
- **ROS Toolbox** (required for live streaming connection to RViz2)

### ROS 2 Side (Linux Ubuntu 22.04 / VM)
- **ROS 2 Humble**
- ROS 2 packages:
  ```bash
  sudo apt update
  sudo apt install -y ros-humble-robot-state-publisher ros-humble-rviz2 ros-humble-xacro
  ```

---

##  Step-by-Step Execution Guide

### 1. URDF Setup (Run once, on Linux)
```bash
cd ~/KinovaCraft/kinova_trajectory_matlab
./setup_urdf.sh
```
This script copies the STL/DAE meshes, configures the TCP frame at the fingertip, and ensures exactly 6 active DOF for MATLAB.

### 2. Launch the RViz2 Visualizer (in Linux terminal)
```bash
cd ~/KinovaCraft
source /opt/ros/humble/setup.bash
colcon build --packages-select trajectory_planner_g6 --symlink-install
source install/setup.bash

export ROS_DOMAIN_ID= <your_VMware_ID>
ros2 launch trajectory_planner_g6 matlab_rviz.launch.py
```
> The Kinova Gen3 6-DOF appears in RViz2 in passive mode, waiting for MATLAB trajectory commands.

### 3. Run Simulation in MATLAB
1. Open MATLAB on your machine.
2. Set the current folder:
   ```matlab
   cd('path/to/KinovaCraft/kinova_trajectory_matlab')
   ```
3. Check parameters in `params.m`:
   ```matlab
   p.shape.type     = 'fish';          % 'fish', 'rose', or 'square'
   p.traj.total_time = 20.0;           % Nominal duration for 100% compliant dynamics
   p.limits.max_velocity = 1.3963;     % URDF manufacturer limit (rad/s)
   p.limits.max_acceleration = 5.0;    % Safe industrial acceleration limit (rad/s^2)
   p.ros2.enable    = true;            % Enable co-simulation
   p.ros2.domain_id = 30;              % Match your Linux VM Domain ID
   p.ros2.nb_tours  = 2;               % Number of trajectory loops in RViz2
   ```
4. Run the main script:
   ```matlab
   main
   ```

---

##  References & Sources

- **Kinova Robotics**: [ros2_kortex (GitHub)](https://github.com/Kinovarobotics/ros2_kortex) for the official Gen3 6-DOF URDF model.
- **MathWorks Robotics System Toolbox**:
  - `importrobot`: Rigid body tree model import.
  - `inverseKinematics`: Numerical IK solver with weighting constraints.
  - `trapveltraj`, `cubicpolytraj`, `quinticpolytraj`, `bsplinepolytraj`: Joint trajectory generation tools.

---

## 👥 Academic Team & Authors

* **Institution:** École Nationale des Sciences Appliquées de Fès (ENSA Fès) — Université Sidi Mohamed Ben Abdellah
* **Program:** 3rd Year Mechatronics Engineering (GMT3), Academic Year 2026/2027
* **Course:** Robot Modeling & Control (Theme 3: Trajectory Planning for Manipulators)
* **Supervised by:** Pr. Mohamed Amine TAHIRI
* **Team (Group G6):**
  * **Aziz IGUIDRE** (Software Architecture, DDS Communication & ROS 2 Co-simulation)
  * **Adam ELASRI** (URDF Kinematic Modeling, TCP Calibration, IK & Interpolation)
  * **Mohammed HANYF** (3D Cartesian Profiles, Trajectory Shaping & Modeling)
  * **Mohamed LAADIMI** (Dynamic Benchmarking, Metrics & Constraints Analysis) 

