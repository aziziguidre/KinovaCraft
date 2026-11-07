function p = params()
%PARAMS Configuration centrale du projet Kinova Gen3 6-DOF
%   Tous les parametres sont regroupes ici.
%   AUCUNE valeur en dur dans main.m.
%
%   Sources :
%   - Kinova Gen3 6-DOF URDF : https://github.com/Kinovarobotics/ros2_kortex
%   - MathWorks Robotics System Toolbox

    %% === Robot ===
    p.robot.urdf_path = fullfile(fileparts(mfilename('fullpath')), ...
                                 'src', 'urdf', 'gen3_6dof.urdf');
    p.robot.num_dof = 6;
    % L'effecteur et les limites de position sont extraits automatiquement
    % de l'objet rigidBodyTree dans load_robot_model.m

    %% === Forme parametrique ===
    p.shape.type       = 'fish';       % Options : 'square', 'fish', 'rose'
    p.shape.radius     = 0.20;         % Rayon/dimension de la forme (m)
    p.shape.n_petals   = 3;            % Nombre de petales (pour 'rose' uniquement)
    p.shape.num_points = 50;           % Nombre de waypoints cibles
    p.shape.center     = [0.45, 0.0, 0.25]; % Centre dans l'espace de travail [X, Y, Z] (m)
    p.shape.orientation_rpy = [pi, 0, 0];   % Orientation effecteur [roll, pitch, yaw] (rad)

    %% === Cinematique inverse (IK) ===
    p.ik.max_error       = 1e-3;       % Erreur max tolerée de position (m)
    p.ik.max_iterations  = 500;
    p.ik.weights         = [0.2, 0.2, 0.2, 1.0, 1.0, 1.0]; % [rot(3), trans(3)]

    %% === Planification de trajectoire ===
    p.traj.total_time = 5.0;          % Duree totale de la trajectoire (s)
    p.traj.dt         = 0.05;          % Pas d'echantillonnage temporel (s) (50 Hz)
    p.traj.methods    = {'trapveltraj', 'cubicpolytraj', ...
                         'quinticpolytraj', 'bsplinepolytraj'};

    %% === Limites dynamiques de securite ===
    % Les limites articulaires de POSITION sont lues directement depuis l'URDF.
    % Limites maximales admissibles pour la validation dynamique :
    p.limits.max_velocity     = 1.5;   % rad/s
    p.limits.max_acceleration = 2.0;   % rad/s^2

    %% === Sorties et Fichiers ===
    p.output.results_dir    = fullfile(fileparts(mfilename('fullpath')), 'results');
    p.output.video_filename = 'trajectoire.mp4';
    p.output.csv_filename   = 'metrics_comparison.csv';
    p.output.fig_format     = '-dpng';
    p.output.fig_resolution = '-r300'; % 300 DPI (qualite publication)
    p.output.video_fps      = 25;      % Images par seconde
    p.output.video_quality  = 95;

    %% === Integration ROS 2 ===
    p.ros2.enable    = true;          % true: publie sur ROS 2; false: desactive
    p.ros2.topic     = '/joint_states';
    p.ros2.domain_id = 30;

end
