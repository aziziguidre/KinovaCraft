function [T, metrics] = evaluate_metrics(traj_results, p, joint_limits, ik_failures)
%EVALUATE_METRICS Calcule les indicateurs de performance et dresse le rapport
%
%   Indicateurs calcules par methode :
%   - Duree totale (s)
%   - Temps de calcul de la planification (s)
%   - Vitesse articulaire maximale (rad/s)
%   - Acceleration articulaire maximale (rad/s^2)
%   - Jerk articulaire maximal (rad/s^3)
%   - Nombre d'echecs IK
%   - Respect des limites dynamiques (booleen)
%
%   Entrees :
%       traj_results - Resultats retournes par plan_trajectories
%       p            - Parametres (params.m)
%       joint_limits - Limites articulaires URDF
%       ik_failures  - Nombre d'echecs IK observes
%   Sorties :
%       T            - Table MATLAB formate
%       metrics      - Structure pour les traces graphiques

    num_m = numel(traj_results);

    method_col    = cell(num_m, 1);
    duration_col  = zeros(num_m, 1);
    comptime_col  = zeros(num_m, 1);
    max_vel_col   = zeros(num_m, 1);
    max_acc_col   = zeros(num_m, 1);
    max_jerk_col  = zeros(num_m, 1);
    ik_fail_col   = zeros(num_m, 1);
    limits_ok_col = cell(num_m, 1);

    for i = 1:num_m
        r  = traj_results(i);
        dt = mean(diff(r.t));

        method_col{i}   = r.method;
        duration_col(i) = r.t(end) - r.t(1);
        comptime_col(i) = r.comp_time;
        max_vel_col(i)  = max(abs(r.qd), [], 'all');
        max_acc_col(i)  = max(abs(r.qdd), [], 'all');

        % Calcul du Jerk (derivee temporelle discrete de l'acceleration)
        if size(r.qdd, 2) > 1 && dt > 0
            jerk = diff(r.qdd, 1, 2) / dt;
            max_jerk_col(i) = max(abs(jerk), [], 'all');
        else
            max_jerk_col(i) = 0.0;
        end

        ik_fail_col(i) = ik_failures;

        % Verification du respect des limites
        is_ok = (max_vel_col(i) <= p.limits.max_velocity) && ...
                (max_acc_col(i) <= p.limits.max_acceleration);
        if is_ok
            limits_ok_col{i} = 'OUI';
        else
            limits_ok_col{i} = 'NON';
        end

        % Stockage struct
        metrics(i).method           = r.method; %#ok<AGROW>
        metrics(i).duration         = duration_col(i);
        metrics(i).comp_time        = comptime_col(i);
        metrics(i).max_velocity     = max_vel_col(i);
        metrics(i).max_acceleration = max_acc_col(i);
        metrics(i).max_jerk         = max_jerk_col(i);
        metrics(i).ik_failures      = ik_failures;
        metrics(i).limits_ok        = is_ok;
    end

    % Creation de la table
    T = table(method_col, duration_col, comptime_col, ...
              max_vel_col, max_acc_col, max_jerk_col, ...
              ik_fail_col, limits_ok_col, ...
              'VariableNames', {'Methode', 'Duree_s', 'Temps_Calcul_s', ...
                                'Vit_Max_rad_s', 'Acc_Max_rad_s2', ...
                                'Jerk_Max_rad_s3', 'Echecs_IK', 'Limites_OK'});

    % Affichage structure dans la console
    fprintf('╔════════════════════════════════════════════════════════════════════════════════════════╗\n');
    fprintf('║                    RAPPORT COMPARATIF DES METHODES DE PLANIFICATION                    ║\n');
    fprintf('╠══════════════════╦══════════╦══════════╦══════════╦══════════╦══════════╦════════╦═══════╣\n');
    fprintf('║ Methode          ║ Duree(s) ║ Calc(ms) ║ VitMax   ║ AccMax   ║ JerkMax  ║ IK Err ║ Valid ║\n');
    fprintf('╠══════════════════╬══════════╬══════════╬══════════╬══════════╬══════════╬════════╬═══════╣\n');
    for i = 1:num_m
        fprintf('║ %-16s ║ %8.2f ║ %8.3f ║ %8.3f ║ %8.3f ║ %8.1f ║ %6d ║ %-5s ║\n', ...
            method_col{i}, duration_col(i), comptime_col(i)*1000, ...
            max_vel_col(i), max_acc_col(i), max_jerk_col(i), ik_fail_col(i), limits_ok_col{i});
    end
    fprintf('╚══════════════════╩══════════╩══════════╩══════════╩══════════╩══════════╩════════╩═══════╝\n');
end
