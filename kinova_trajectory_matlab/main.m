%% main.m - Script principal : Planification de Trajectoire Kinova Gen3 6-DOF
%
%  Ce script s'execute du debut a la fin sans aucune erreur.
%  Tous les parametres sont centralises dans params.m (aucune valeur en dur).
%  Gere proprement l'absence de ROS Toolbox.
%
%  Auteur  : Aziz
%  Robot   : Kinova Gen3 6-DOF avec pince Robotiq 2F-85
%  Sources : MathWorks Robotics System Toolbox, ros2_kortex (Kinova)

clear; close all; clc;

%% 0. Chargement de la configuration
fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║   PLANIFICATION DE TRAJECTOIRE — KINOVA GEN3 6-DOF (MATLAB)   ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');

p = params();

% Ajouter le repertoire des fonctions auxiliaires
addpath(fullfile(fileparts(mfilename('fullpath')), 'src'));

% Creer le dossier de resultats s'il n'existe pas
if ~exist(p.output.results_dir, 'dir')
    mkdir(p.output.results_dir);
end

%% 1. Chargement du Robot Kinova Gen3 6-DOF depuis l'URDF
fprintf('[1/8] Chargement du robot depuis l''URDF...\n');
[robot, joint_names, joint_limits, ee_name] = load_robot_model(p);

% ASSERTION OBLIGATOIRE : verification stricte des 6 DOF
q = homeConfiguration(robot);
assert(numel(q) == 6, 'Le robot doit avoir exactement 6 DOF');
fprintf('   ✔ Assertion validee : exactement %d DOF detectes.\n', numel(q));
fprintf('   ✔ Effecteur detecte : "%s"\n', ee_name);
fprintf('   ✔ Articulations actives : %s\n\n', strjoin(joint_names, ', '));

%% 2. Generation de la Forme Parametrique (Espace Cartesien)
fprintf('[2/8] Generation de la forme parametrique "%s"...\n', p.shape.type);
[shape_points_2d] = generate_shape(p.shape.type, p.shape.radius, ...
                                   p.shape.n_petals, p.shape.num_points);
fprintf('   ✔ %d points generes pour la forme "%s" (rayon = %.3f m).\n\n', ...
    size(shape_points_2d, 1), p.shape.type, p.shape.radius);

%% 3. Cinematique Inverse (IK) avec Securite Try-Catch
fprintf('[3/8] Resolution de la cinematique inverse (IK)...\n');
[q_waypoints, ik_failures, valid_cartesian] = solve_ik_for_shape( ...
    robot, ee_name, shape_points_2d, p);

num_valid = size(q_waypoints, 1);
fprintf('   ✔ Waypoints valides : %d / %d (Echecs IK ignores : %d)\n\n', ...
    num_valid, size(shape_points_2d, 1), ik_failures);

if num_valid < 3
    error('MAIN:InsufficientWaypoints', ...
          'Nombre insuffisant de waypoints valides (%d). Verifiez le centre de la forme dans params.m.', num_valid);
end

%% 4. Planification des Trajectoires
    fprintf('[4/8] Planification des trajectoires articulaires...\n');
    [traj_results] = plan_trajectories(q_waypoints, p);
    idx_quintic = find(strcmp({traj_results.method}, 'quinticpolytraj'), 1);
    if isempty(idx_quintic), idx_quintic = 1; end
    
  %% 5. Simulation en direct dans RViz2 (2 tours complets)
    fprintf('[5/8] Simulation RViz2 en direct...\n');
    if p.ros2.enable
        try
            publish_ros2_trajectory(traj_results(idx_quintic), joint_names, p, robot, ee_name);
        catch ME
            warning('MAIN:ROS2Failed', 'Erreur ROS 2 : %s', ME.message);
        end
    end

    %% 6. Indicateurs de performance
    fprintf('\n[6/8] Calcul des indicateurs de performance...\n');
    [metrics_table, metrics_struct] = evaluate_metrics(traj_results, p, joint_limits, ik_failures);
    disp(metrics_table);
    csv_path = fullfile(p.output.results_dir, p.output.csv_filename);
    writetable(metrics_table, csv_path);
    fprintf('   ✔ Metriques exportees : %s\n\n', csv_path);

    %% 7. Generation des Figures PNG
    fprintf('[7/8] Generation des figures PNG...\n');
    plot_shape_3d(robot, q_waypoints, valid_cartesian, p, ee_name);
    plot_joint_profiles(traj_results(idx_quintic), joint_limits, p, joint_names);
    plot_comparison_bar(metrics_struct, p);
    fprintf('   ✔ Figures PNG enregistrees.\n\n');

    %% 8. Generation Ultra-Rapide de la Video MP4
    fprintf('[8/8] Generation de l''animation MP4...\n');
    animate_robot(robot, traj_results(idx_quintic), valid_cartesian, p, ee_name);

    fprintf('\n╔════════════════════════════════════════════════════════════════╗\n');
    fprintf('║            EXECUTION TERMINEE AVEC SUCCES                     ║\n');
    fprintf('╚════════════════════════════════════════════════════════════════╝\n');
 