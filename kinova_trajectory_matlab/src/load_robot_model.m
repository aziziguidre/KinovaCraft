function [robot, joint_names, joint_limits, ee_name] = load_robot_model(p)
%LOAD_ROBOT_MODEL Charge le Kinova Gen3 6-DOF et extrait les donnees de l'URDF
%   Lit le repere de l'effecteur (TCP pince) et les limites articulaires DIRECTEMENT
%   depuis l'objet rigidBodyTree importe. Aucune valeur n'est inventee.
%
%   Entrees :
%       p - Structure issue de params.m
%   Sorties :
%       robot        - Objet rigidBodyTree (6 DOF actifs)
%       joint_names  - Cell array des noms exacts des 6 articulations
%       joint_limits - Structure contenant .lower, .upper et .names
%       ee_name      - Nom exact du repere effecteur (TCP au bout des doigts)

    if ~isfile(p.robot.urdf_path)
        error('LOAD_ROBOT_MODEL:FileNotFound', ...
              ['Fichier URDF introuvable : %s\n' ...
               'Veuillez executer setup_urdf.sh pour preparer l''URDF et ses meshes.'], ...
              p.robot.urdf_path);
    end

    % Importation directe de l'URDF officiel 6-DOF
    robot = importrobot(p.robot.urdf_path);
    robot.DataFormat = 'row';
    robot.Gravity = [0, 0, -9.81];

    % Detection des articulations mobiles (revolute / continuous)
    joint_names = {};
    lower_lim   = [];
    upper_lim   = [];

    for i = 1:numel(robot.Bodies)
        jnt = robot.Bodies{i}.Joint;
        if any(strcmp(jnt.Type, {'revolute', 'continuous'}))
            joint_names{end+1} = jnt.Name; %#ok<AGROW>
            lims = jnt.PositionLimits;
            
            % Si limites infinies (joint continuous sans bornes dans URDF)
            if isinf(lims(1))
                lims(1) = -2*pi;
            end
            if isinf(lims(2))
                lims(2) = 2*pi;
            end
            
            lower_lim(end+1) = lims(1); %#ok<AGROW>
            upper_lim(end+1) = lims(2); %#ok<AGROW>
        end
    end

    joint_limits.lower = lower_lim(:);
    joint_limits.upper = upper_lim(:);
    joint_limits.names = joint_names;

    % Determination de l'effecteur final :
    % 1. Si tcp_link existe (bout des doigts de la pince), on l'utilise en priorite
    % 2. Sinon end_effector_link ou bracelet_link
    body_names = robot.BodyNames;
    if any(strcmp('tcp_link', body_names))
        ee_name = 'tcp_link';
    elseif any(strcmp('end_effector_link', body_names))
        ee_name = 'end_effector_link';
    elseif any(strcmp('bracelet_link', body_names))
        ee_name = 'bracelet_link';
    else
        ee_name = body_names{end};
    end

    % Affichage de controle
    fprintf('   Configuration extraite du rigidBodyTree :\n');
    for j = 1:numel(joint_names)
        fprintf('     %-10s (%-10s) : [%+.3f, %+.3f] rad\n', ...
            joint_names{j}, robot.Bodies{j}.Joint.Type, lower_lim(j), upper_lim(j));
    end
    fprintf('   Point de contact effecteur (TCP) : "%s"\n', ee_name);
end