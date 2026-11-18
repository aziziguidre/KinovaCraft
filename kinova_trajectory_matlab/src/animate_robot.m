 function animate_robot(robot, traj, valid_cartesian, p, ee_name)
    %ANIMATE_ROBOT Animation video haute vitesse avec fermeture/ouverture visible de la pince
    
        video_path = fullfile(p.output.results_dir, p.output.video_filename);
    
        v = VideoWriter(video_path, 'MPEG-4');
        v.FrameRate = p.output.video_fps;
        v.Quality   = p.output.video_quality;
        open(v);
    
        total_pts = size(traj.q, 2);
        target_frames = 75;
        frame_indices = round(linspace(1, total_pts, target_frames));
        num_frames = numel(frame_indices);
    
        fprintf('   Rendu video avec pince active (%d images)...\n', num_frames);
    
        fig = figure('Position', [100, 100, 960, 540], 'Color', 'w', 'Visible', 'off');
        ax = axes('Parent', fig);
    
        trail_x = zeros(1, num_frames);
        trail_y = zeros(1, num_frames);
        trail_z = zeros(1, num_frames);
    
        % Recuperer les liaisons de la pince pour les animer
        body_l_knuckle = [];
        body_r_knuckle = [];
        body_names = robot.BodyNames;
        if any(strcmp('robotiq_85_left_knuckle_link', body_names))
            body_l_knuckle = robot.getBody('robotiq_85_left_knuckle_link');
        end
        if any(strcmp('robotiq_85_right_knuckle_link', body_names))
            body_r_knuckle = robot.getBody('robotiq_85_right_knuckle_link');
        end
    
        % Origines neutres des articulations
        T_orig_l = eye(4);
        T_orig_l(1:3, 4) = [0.03060114; 0.0; 0.05490452];
        T_orig_r = eye(4);
        T_orig_r(1:3, 4) = [-0.03060114; 0.0; 0.05490452];
    
        for fi = 1:num_frames
            idx = frame_indices(fi);
            q_curr = traj.q(:, idx)';
    
            % 1. Fermeture au point 1, ouverture au dernier point dans MATLAB
            if fi == num_frames
                theta = 0.0;       % Pince ouverte au dernier point
            else
                theta = 0.7929;    % Pince fermee pendant tout le trace de la rose
            end
    
            % Appliquer la rotation de fermeture sur les doigts du robot
            if ~isempty(body_l_knuckle) && ~isempty(body_r_knuckle)
                R_l = eul2rotm([0, -theta, 0], 'XYZ');
                T_l = T_orig_l;
                T_l(1:3, 1:3) = R_l;
                setFixedTransform(body_l_knuckle.Joint, T_l);
    
                R_r = eul2rotm([0, theta, 0], 'XYZ');
                T_r = T_orig_r;
                T_r(1:3, 1:3) = R_r;
                setFixedTransform(body_r_knuckle.Joint, T_r);
            end
    
            cla(ax);
    
            % Dessin du robot avec pince fermee
            show(robot, q_curr, 'Frames', 'off', 'PreservePlot', false, ...
                 'Parent', ax, 'Visuals', 'on', 'FastUpdate', 1);
            hold(ax, 'on');
    
            % Trace cible en arriere-plan
            if ~isempty(valid_cartesian)
                plot3(ax, valid_cartesian(:,1), valid_cartesian(:,2), valid_cartesian(:,3), ...
                      '--', 'Color', [0.6, 0.6, 0.6], 'LineWidth', 1.0, 'HandleVisibility', 'off');
            end
    
            % Position au bout des doigts (TCP)
            T_now = getTransform(robot, q_curr, ee_name);
            pos_ee = T_now(1:3, 4);
            trail_x(fi) = pos_ee(1);
            trail_y(fi) = pos_ee(2);
            trail_z(fi) = pos_ee(3);
    
            % Ligne orange sortant directement des mors fermes
            plot3(ax, trail_x(1:fi), trail_y(1:fi), trail_z(1:fi), ...
                  '-', 'Color', [1.0, 0.45, 0.0], 'LineWidth', 2.5, 'HandleVisibility', 'off');

            scatter3(ax, pos_ee(1), pos_ee(2), pos_ee(3), 70, [0.9, 0.1, 0.1], ...
                     'filled', 'HandleVisibility', 'off');

            title(ax, sprintf('Kinova Gen3 6-DOF - %s', traj.method), 'FontSize', 12, 'FontWeight', 'bold');
            xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');
            view(ax, 45, 25);
            axis(ax, 'equal');
            grid(ax, 'on');
            hold(ax, 'off');

            drawnow limitrate;
            writeVideo(v, getframe(fig));
        end

        % Reinitialiser la pince en position ouverte a la fin
        if ~isempty(body_l_knuckle) && ~isempty(body_r_knuckle)
            setFixedTransform(body_l_knuckle.Joint, T_orig_l);
            setFixedTransform(body_r_knuckle.Joint, T_orig_r);
        end

        close(v);
        close(fig);
        fprintf('   ✔ Video MP4 generee avec pince fermee au contact : %s\n', video_path);
    end
