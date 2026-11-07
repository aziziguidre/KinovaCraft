function plot_comparison_bar(metrics, p)
%PLOT_COMPARISON_BAR Trace un bar chart comparant les 4 methodes
%   Compare vitesse max, acceleration max, jerk max et temps de calcul.
%   Tous les graphiques possedent titre, axes nommes avec unites et limites.
%
%   Entrees :
%       metrics - Structure contenant les metriques evaluees
%       p       - Structure des parametres

    num_m = numel(metrics);
    labels = cell(num_m, 1);
    for i = 1:num_m
        labels{i} = strrep(metrics(i).method, 'polytraj', '');
        labels{i} = strrep(labels{i}, 'traj', '');
    end

    max_v = [metrics.max_velocity];
    max_a = [metrics.max_acceleration];
    max_j = [metrics.max_jerk];
    t_c   = [metrics.comp_time] * 1000; % ms

    fig = figure('Name', 'Comparatif des Methodes', ...
                 'Position', [100, 100, 1000, 750], ...
                 'Color', 'w', ...
                 'Visible', 'off');

    bar_colors = [0.20, 0.50, 0.85;   % Bleu
                  0.90, 0.55, 0.15;   % Orange
                  0.30, 0.75, 0.35;   % Vert
                  0.80, 0.30, 0.55];  % Mauve

    %% 1. Vitesse max
    subplot(2, 2, 1);
    b1 = bar(max_v, 'FaceColor', 'flat');
    for k = 1:num_m
        b1.CData(k,:) = bar_colors(k,:);
    end
    hold on;
    yline(p.limits.max_velocity, '--r', 'LineWidth', 1.4, ...
          'DisplayName', sprintf('Limite (%.1f rad/s)', p.limits.max_velocity));
    hold off;
    set(gca, 'XTickLabel', labels, 'FontSize', 9);
    title('Vitesse Articulaire Maximale', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel('Vitesse max (rad/s)', 'FontSize', 10);
    legend('Location', 'northeast', 'FontSize', 8);
    grid on;

    %% 2. Acceleration max
    subplot(2, 2, 2);
    b2 = bar(max_a, 'FaceColor', 'flat');
    for k = 1:num_m
        b2.CData(k,:) = bar_colors(k,:);
    end
    hold on;
    yline(p.limits.max_acceleration, '--r', 'LineWidth', 1.4, ...
          'DisplayName', sprintf('Limite (%.1f rad/s^2)', p.limits.max_acceleration));
    hold off;
    set(gca, 'XTickLabel', labels, 'FontSize', 9);
    title('Acceleration Articulaire Maximale', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel('Acceleration max (rad/s^2)', 'FontSize', 10);
    legend('Location', 'northeast', 'FontSize', 8);
    grid on;

    %% 3. Jerk max
    subplot(2, 2, 3);
    b3 = bar(max_j, 'FaceColor', 'flat');
    for k = 1:num_m
        b3.CData(k,:) = bar_colors(k,:);
    end
    set(gca, 'XTickLabel', labels, 'FontSize', 9);
    title('Jerk Articulaire Maximal', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel('Jerk max (rad/s^3)', 'FontSize', 10);
    grid on;

    %% 4. Temps de calcul
    subplot(2, 2, 4);
    b4 = bar(t_c, 'FaceColor', 'flat');
    for k = 1:num_m
        b4.CData(k,:) = bar_colors(k,:);
    end
    set(gca, 'XTickLabel', labels, 'FontSize', 9);
    title('Temps de Calcul Algorithmique', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel('Temps (ms)', 'FontSize', 10);
    grid on;

    sgtitle('Comparaison Quantitative des Methodes de Planification', ...
            'FontSize', 14, 'FontWeight', 'bold');

    % Sauvegarde PNG 300 DPI
    out_file = fullfile(p.output.results_dir, 'comparaison_methodes.png');
    print(fig, out_file, p.output.fig_format, p.output.fig_resolution);
    close(fig);
    fprintf('   ✔ Figure enregistree : %s\n', out_file);
end
