function [q_waypoints, ik_failures, valid_cartesian] = solve_ik_for_shape( ...
    robot, ee_name, shape_points, p)
%SOLVE_IK_FOR_SHAPE Resout l'IK pour chaque point avec securite try-catch
%   Si l'IK echoue ou si l'erreur de position > p.ik.max_error, le point est
%   ignore afin d'eviter tout plantage de main.m.
%
%   Entrees :
%       robot        - Objet rigidBodyTree
%       ee_name      - Nom de l'effecteur final
%       shape_points - Matrice (N x 3) des waypoints relatifs generes
%       p            - Structure des parametres (params.m)
%   Sorties :
%       q_waypoints     - Matrice (M x 6) des configurations articulaires valides
%       ik_failures     - Nombre d'echecs IK constates
%       valid_cartesian - Matrice (M x 3) des coordonnees cartesiennes correspondantes

    % Creation de l'objet de cinematique inverse
    ik = inverseKinematics('RigidBodyTree', robot);
    ik.SolverParameters.MaxIterations = p.ik.max_iterations;
    ik.SolverParameters.GradientTolerance = 1e-6;
    ik.SolverParameters.SolutionTolerance = 1e-5;

    weights = p.ik.weights;
    R_target = eul2rotm(p.shape.orientation_rpy, 'XYZ');

    q_prev = homeConfiguration(robot);

    q_waypoints     = [];
    valid_cartesian = [];
    ik_failures     = 0;

    num_pts = size(shape_points, 1);
    fprintf('   Progression IK : ');

    for i = 1:num_pts
        % Coordonnees absolues dans l'espace de travail
        pos_target = p.shape.center(:)' + shape_points(i, :);

        T_target = eye(4);
        T_target(1:3, 1:3) = R_target;
        T_target(1:3, 4)   = pos_target(:);

        try
            % Appel robuste a l'IK
            [q_sol, solInfo] = ik(ee_name, T_target, weights, q_prev);

            % Verification rigoureuse de l'erreur cartesienne
            T_actual  = getTransform(robot, q_sol, ee_name);
            pos_error = norm(T_actual(1:3, 4) - T_target(1:3, 4));

            if pos_error > p.ik.max_error
                % Erreur superieure au seuil : point ignore
                ik_failures = ik_failures + 1;
                fprintf('x');
            else
                % Point valide accepte
                q_waypoints     = [q_waypoints; q_sol]; %#ok<AGROW>
                valid_cartesian = [valid_cartesian; pos_target]; %#ok<AGROW>
                q_prev          = q_sol;
                fprintf('.');
            end

        catch ME %#ok<NASGU>
            % Securite : tout echec IK est intercepte sans planter
            ik_failures = ik_failures + 1;
            fprintf('X');
        end

        if mod(i, 20) == 0
            fprintf(' [%d/%d]\n   ', i, num_pts);
        end
    end
    fprintf(' Termine.\n');
end
