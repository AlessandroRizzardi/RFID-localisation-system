phase_measured = robot.phaseMeasured(tag_position, lambda , sigma_phi);
        phase_history(k,1) = phase_measured;

        % Prediction and Correction EKF
        for l = 1:nM
            EKF_instances(l).EKF_predict(odometry_estimation, d);
            EKF_instances(l).EKF_correct(K, sigma_phi, phase_measured);
            
            if method_paper == false
                weights_vec(l) = EKF_instances(l).weight;
            end

            % Save the state history of each EKF instance
            EKF_instances(l).state_history{k,1} = EKF_instances(l).x;

            % if isnan(EKF_instances(l).state_history{k,1}) == true
            %     fprintf('Error state history\n');
            % end

        end

        % Correction of non-positive range estimation
        for l = 1:nM
            if EKF_instances(l).x(1) <= 10^-6
                EKF_instances(l).x(1) = max(abs(EKF_instances(l).x(1)),10^-6);
                EKF_instances(l).x(2) = EKF_instances(l).x(2) + pi;
            end
            EKF_instances(l).x(2) = atan2(sin(EKF_instances(l).x(2)),cos(EKF_instances(l).x(2)));
        end

        target_point(1) = best_tag_estimation_x + tag_window(1) + (tag_window(2)-tag_window(1))*rand();
        target_point(2) = best_tag_estimation_y + tag_window(1) + (tag_window(2)-tag_window(1))*rand();

        [v,omega] = greedy_controller(Kp_v1,Kp_w1, target_point(1),target_point(2),robot.x_est);
        
        x_next = robot.dynamics(v,omega);
        dynamics_history{k,1} = x_next;

        odometry_estimation = robot.odometry_step(v,omega);
        odometry_history{k,1} = robot.x_est;