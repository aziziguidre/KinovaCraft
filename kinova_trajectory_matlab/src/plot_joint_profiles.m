function plot_joint_profiles(traj, joint_limits, p, joint_names)
%PLOT_JOINT_PROFILES Trace les profils q(t), qdot(t), qddot(t) avec limites
%   Conforme aux exigences : 3 sous-figures avec unites, legendes et limites rouges.
%
%   Entrees :
%       traj         - Structure de la trajectoire (quinticpolytraj)
%       joint_limits - Structure des limites articulaires issues de l'URDF
%       p            - Structure des parametres
%       joint_names  - Noms des 6 articulations du Gen3

    num_dof = size(traj.q, 1);
    t = traj.t;
    colors = lines(num_dof);

    fig = figure('Name', sprintf('Profils Articulaires - %s', traj.method), ...
                 'Position', [100, 50, 1100, 850], ...
                 'Color', 'w', ...
                 'Visible', 'off');

    %% Subplot 1 : Positions articulaires q(t)
    subplot(3, 1, 1);
    hold on;
    for j = 1:num_dof
        plot(t, traj.q(j,:), 'Color', colors(j,:), 'LineWidth', 1.6, ...
             'DisplayName', joint_names{j});
    end
    % Lignes de limites de position (issues de l'URDF)
    for j = 1:num_dof
        yline(joint_limits.lower(j), ':', 'Color', [0.8, 0.2, 0.2], ...
              'LineWidth', 0.9, 'HandleVisibility', 'off');
        yline(joint_limits.upper(j), ':', 'Color', [0.8, 0.2, 0.2], ...
              'LineWidth', 0.9, 'HandleVisibility', 'off');
    end
    yline(NaN, ':r', 'LineWidth', 1.2, 'DisplayName', 'Limites Position (URDF)');
    title(sprintf('Positions Articulaires q(t) — %s', strrep(traj.method, '_', '\_')), ...
          'FontSize', 12, 'FontWeight', 'bold');
    xlabel('Temps (s)', 'FontSize', 10);
    ylabel('Position (rad)', 'FontSize', 10);
    legend('Location', 'eastoutside', 'FontSize', 8);
    grid on;
    hold off;

    %% Subplot 2 : Vitesses articulaires dq/dt
    subplot(3, 1, 2);
    hold on;
    for j = 1:num_dof
        plot(t, traj.qd(j,:), 'Color', colors(j,:), 'LineWidth', 1.6, ...
             'DisplayName', joint_names{j});
    end
    % Limites de securite (vitesse max)
    yline( p.limits.max_velocity, '--r', 'LineWidth', 1.5, ...
           'DisplayName', sprintf('Limite Vitesse (±%.1f rad/s)', p.limits.max_velocity));
    yline(-p.limits.max_velocity, '--r', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    title(sprintf('Vitesses Articulaires dq(t)/dt — %s', strrep(traj.method, '_', '\_')), ...
          'FontSize', 12, 'FontWeight', 'bold');
    xlabel('Temps (s)', 'FontSize', 10);
    ylabel('Vitesse (rad/s)', 'FontSize', 10);
    legend('Location', 'eastoutside', 'FontSize', 8);
    grid on;
    hold off;

    %% Subplot 3 : Accelerations articulaires d2q/dt2
    subplot(3, 1, 3);
    hold on;
    for j = 1:num_dof
        plot(t, traj.qdd(j,:), 'Color', colors(j,:), 'LineWidth', 1.6, ...
             'DisplayName', joint_names{j});
    end
    % Limites de securite (acceleration max)
    yline( p.limits.max_acceleration, '--r', 'LineWidth', 1.5, ...
           'DisplayName', sprintf('Limite Acceleration (±%.1f rad/s^2)', p.limits.max_acceleration));
    yline(-p.limits.max_acceleration, '--r', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    title(sprintf('Accelerations Articulaires d^2q(t)/dt^2 — %s', strrep(traj.method, '_', '\_')), ...
          'FontSize', 12, 'FontWeight', 'bold');
    xlabel('Temps (s)', 'FontSize', 10);
    ylabel('Acceleration (rad/s^2)', 'FontSize', 10);
    legend('Location', 'eastoutside', 'FontSize', 8);
    grid on;
    hold off;

    sgtitle('Profils Articulaires Complets — Methode Quintique', ...
            'FontSize', 14, 'FontWeight', 'bold');

    % Sauvegarde PNG 300 DPI
    out_file = fullfile(p.output.results_dir, 'profils_articulaires.png');
    print(fig, out_file, p.output.fig_format, p.output.fig_resolution);
    close(fig);
    fprintf('   ✔ Figure enregistree : %s\n', out_file);
end
