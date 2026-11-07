function [results] = plan_trajectories(q_waypoints, p)
%PLAN_TRAJECTORIES Planifie et compare 4 methodes d'interpolation articulaire
%   Methodes comparees :
%   - trapveltraj    : Trajectoire trapezoidale de vitesse
%   - cubicpolytraj  : Polynome cubique (C1)
%   - quinticpolytraj: Polynome quintique (C2)
%   - bsplinepolytraj: Courbe B-spline uniforme (C2)
%
%   Entrees :
%       q_waypoints - Matrice (M x 6) des waypoints articulaires valides
%       p           - Structure des parametres (params.m)
%   Sortie :
%       results     - Tableau de structures avec .method, .q, .qd, .qdd, .t, .comp_time

    % MATLAB attend les waypoints sous forme (num_dof x num_waypoints)
    wpts = q_waypoints';
    num_wp = size(wpts, 2);

    % Vecteur temporel des waypoints et vecteur d'echantillonnage
    t_wp = linspace(0, p.traj.total_time, num_wp);
    t_samples = 0 : p.traj.dt : p.traj.total_time;
    num_samples = numel(t_samples);

    methods = p.traj.methods;
    results = struct('method', {}, 'q', {}, 'qd', {}, 'qdd', {}, ...
                     't', {}, 'comp_time', {});

    for i = 1:numel(methods)
        m = methods{i};
        tic_start = tic;

        try
            switch m
                case 'trapveltraj'
                    % Profil trapezoidal de vitesse
                    [q, qd, qdd, ~] = trapveltraj(wpts, num_samples, ...
                                                 'EndTime', p.traj.total_time);
                    t = t_samples;

                case 'cubicpolytraj'
                    % Polynome cubique par morceaux (C1)
                    [q, qd, qdd, ~] = cubicpolytraj(wpts, t_wp, t_samples);
                    t = t_samples;

                case 'quinticpolytraj'
                    % Polynome quintique par morceaux (C2)
                    [q, qd, qdd, ~] = quinticpolytraj(wpts, t_wp, t_samples);
                    t = t_samples;

                case 'bsplinepolytraj'
                    % B-spline uniforme (C2)
                    [q, qd, qdd, ~] = bsplinepolytraj(wpts, [0, p.traj.total_time], t_samples);
                    t = t_samples;

                otherwise
                    warning('PLAN_TRAJECTORIES:UnknownMethod', 'Methode "%s" non reconnue.', m);
                    continue;
            end

            elapsed = toc(tic_start);

            % Stockage des resultats
            idx = numel(results) + 1;
            results(idx).method    = m;
            results(idx).q         = q;
            results(idx).qd        = qd;
            results(idx).qdd       = qdd;
            results(idx).t         = t;
            results(idx).comp_time = elapsed;

        catch ME
            elapsed = toc(tic_start);
            warning('PLAN_TRAJECTORIES:ExecutionFailed', ...
                    'Echec de la planification avec "%s" : %s', m, ME.message);

            idx = numel(results) + 1;
            results(idx).method    = m;
            results(idx).q         = zeros(size(wpts, 1), num_samples);
            results(idx).qd        = zeros(size(wpts, 1), num_samples);
            results(idx).qdd       = zeros(size(wpts, 1), num_samples);
            results(idx).t         = t_samples;
            results(idx).comp_time = elapsed;
        end
    end
end
