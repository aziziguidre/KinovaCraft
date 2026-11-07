function plot_shape_3d(robot, q_waypoints, valid_cartesian, p, ee_name)
    
        fig = figure('Position', [100, 100, 960, 720], 'Color', 'w', 'Visible', 'off');
        ax = axes('Parent', fig);
    
        show(robot, q_waypoints(1, :), 'Frames', 'off', 'PreservePlot', false, 'Parent', ax);
        hold(ax, 'on');
    
        h_path = plot3(ax, valid_cartesian(:,1), valid_cartesian(:,2), valid_cartesian(:,3), ...
                       '-', 'Color', [0.1, 0.45, 0.85], 'LineWidth', 2.5);
    
        plot3(ax, [valid_cartesian(end,1), valid_cartesian(1,1)], ...
                 [valid_cartesian(end,2), valid_cartesian(1,2)], ...
                 [valid_cartesian(end,3), valid_cartesian(1,3)], ...
              '--', 'Color', [0.1, 0.45, 0.85], 'LineWidth', 1.2, 'HandleVisibility', 'off');
    
        num_wp = size(valid_cartesian, 1);
        h_start = scatter3(ax, valid_cartesian(1,1), valid_cartesian(1,2), valid_cartesian(1,3), ...
                           120, [0.1, 0.75, 0.2], 'filled');
        
        if num_wp > 2
            h_inter = scatter3(ax, valid_cartesian(2:end-1, 1), valid_cartesian(2:end-1, 2), ...
                               valid_cartesian(2:end-1, 3), 35, [0.0, 0.5, 0.9], 'filled');
        end

        h_end = scatter3(ax, valid_cartesian(end,1), valid_cartesian(end,2), valid_cartesian(end,3), ...
                         120, [0.85, 0.15, 0.15], 'filled');

        title(ax, sprintf('Forme 3D : %s (%d Waypoints)', upper(p.shape.type), num_wp), ...
              'FontSize', 13, 'FontWeight', 'bold');
        xlabel(ax, 'X (m)');
        ylabel(ax, 'Y (m)');
        zlabel(ax, 'Z (m)');

        if num_wp > 2
            legend([h_path, h_start, h_inter, h_end], ...
                   {'Trajectoire', 'Depart', 'Intermediaires', 'Fin'}, ...
                   'Location', 'northeast');
        else
            legend([h_path, h_start, h_end], {'Trajectoire', 'Depart', 'Fin'}, 'Location', 'northeast');
        end

        grid(ax, 'on');
        view(ax, 45, 30);
        axis(ax, 'equal');
        hold(ax, 'off');

        out_file = fullfile(p.output.results_dir, 'forme_3d.png');
        print(fig, out_file, p.output.fig_format, p.output.fig_resolution);
        close(fig);
        fprintf('   ✔ Figure enregistree : %s\n', out_file);
    end
