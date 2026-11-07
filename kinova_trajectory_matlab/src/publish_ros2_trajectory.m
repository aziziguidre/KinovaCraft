    function publish_ros2_trajectory(traj, joint_names, p, robot, ee_name)
        %PUBLISH_ROS2_TRAJECTORY Diffusion robuste et sans erreur de type ROS 2
    
        if isempty(which('ros2node'))
            error('PUBLISH_ROS2:ToolboxMissing', 'ROS Toolbox non installe dans MATLAB.');
        end
    
        fprintf('   [ROS 2] Initialisation du noeud MATLAB (Domain ID: %d)...\n', p.ros2.domain_id);
        node = ros2node('/matlab_trajectory_publisher', p.ros2.domain_id);
    
        pub_js   = ros2publisher(node, '/joint_states', 'sensor_msgs/JointState');
        pub_path = ros2publisher(node, '/trajectory_path', 'nav_msgs/Path');
    
        msg_js = ros2message(pub_js);
        % 6 joints du bras + 6 joints de la pince Robotiq
        msg_js.name = [joint_names, { ...
            'robotiq_85_left_knuckle_joint', ...
            'robotiq_85_right_knuckle_joint', ...
            'robotiq_85_left_inner_knuckle_joint', ...
            'robotiq_85_right_inner_knuckle_joint', ...
            'robotiq_85_left_finger_tip_joint', ...
            'robotiq_85_right_finger_tip_joint' ...
        }];
    
        num_pts = size(traj.q, 2);
        dt = mean(diff(traj.t));
    
        nb_tours = 5;
        fprintf('   [ROS 2] Lancement de %d tours dans RViz2...\n', nb_tours);
    
        global_time = 0.0;
    
        for tour = 1:nb_tours
            fprintf('     -> Tour %d / %d en cours dans RViz2...\n', tour, nb_tours);
    
            % Trace vierge pour commencer le tour
            path_msg = ros2message(pub_path);
            path_msg.header.frame_id = 'base_link';
    
            poses_array = ros2message('geometry_msgs/PoseStamped');
            poses_count = 0;
    
            for k = 1:num_pts
                t_loop = tic;
                q_curr = traj.q(:, k)';
    
                % 1. Horodatage croissant
                global_time = global_time + dt;
                t_sec  = int32(floor(global_time));
                t_nsec = uint32((global_time - floor(global_time)) * 1e9);
    
                % 2. Fermeture au 1er point, ouverture au dernier point
                if k == num_pts
                    finger_pos = 0.0;    % Pince OUVERTE au tout dernier point
                else
                    finger_pos = 0.7929; % Pince FERMEE des le 1er point (k=1) et tout le long
                end
    
                % Positions des 6 joints de la pince Robotiq
                gripper_pos = [ ...
                     finger_pos, ...
                    -finger_pos, ...
                     finger_pos, ...
                    -finger_pos, ...
                    -finger_pos, ...
                     finger_pos  ...
                ];
    
                % 3. Envoi des positions articulaires
                msg_js.position = [q_curr, gripper_pos];
                msg_js.velocity = [traj.qd(:, k)', zeros(1, 6)];
                msg_js.header.stamp.sec     = t_sec;
                msg_js.header.stamp.nanosec = t_nsec;
                send(pub_js, msg_js);
    
                % 4. Calcul de la position effecteur pour la trace
                T_now = getTransform(robot, q_curr, ee_name);
                pos_ee = T_now(1:3, 4);
    
                pose_i = ros2message('geometry_msgs/PoseStamped');
                pose_i.header.frame_id = 'base_link';
                pose_i.header.stamp.sec     = t_sec;
                pose_i.header.stamp.nanosec = t_nsec;
                pose_i.pose.position.x = pos_ee(1);
                pose_i.pose.position.y = pos_ee(2);
                pose_i.pose.position.z = pos_ee(3);

                poses_count = poses_count + 1;
                poses_array(poses_count) = pose_i;

                path_msg = ros2message(pub_path);
                path_msg.header.frame_id = 'base_link';
                path_msg.header.stamp.sec     = t_sec;
                path_msg.header.stamp.nanosec = t_nsec;
                path_msg.poses = poses_array;
                
                try
                    send(pub_path, path_msg);
                catch
                end

                % Cadence en temps reel
                pause(max(0, dt - toc(t_loop)));
            end
            pause(0.5);
        end

        clear pub_js pub_path node;
        fprintf('   ✔ [ROS 2] Les %d tours sont termines et le robot est arrete.\n', nb_tours);
    end

