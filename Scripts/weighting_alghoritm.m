if steps_in_range >= Ns
    % Weighing Step
    
    if method_paper == true
        for l = 1:nM
            %display(k)
            weights_tmp(l) = EKF_instances(l).EKF_weight_tmp(k, odometry_history, phase_history, Ns, weights_prev(l), c1, c2, K,l);
        end

        eta = compute_eta(weights_tmp);

        for l = 1:nM
            weights(l) = EKF_instances(l).EKF_weight(weights_tmp(l), eta);
        end

        weights_prev = weights;

        [max_value,instance_selected] = max(weights);
    else

        [max_value,instance_selected] = max(weights_vec);
    end

    rho_est = EKF_instances(instance_selected).x(1);
    beta_est = EKF_instances(instance_selected).x(2);

    best_state_estimate = [rho_est;beta_est];

    best_tag_estimation_x = robot.x_est(1) + best_state_estimate(1)*cos(robot.x_est(3) - best_state_estimate(2));
    best_tag_estimation_y = robot.x_est(2) + best_state_estimate(1)*sin(robot.x_est(3) - best_state_estimate(2));
    

else % we don't have enough measurements to weigh the EKF instances --> final estimates are the ones of the last EKF instance

    rho_est  = EKF_instances(nM).x(1);
    beta_est = EKF_instances(nM).x(2);

    best_state_estimate = [rho_est;beta_est];

    best_tag_estimation_x = robot.x_est(1) + best_state_estimate(1)*cos(robot.x_est(3) - best_state_estimate(2));
    best_tag_estimation_y = robot.x_est(2) + best_state_estimate(1)*sin(robot.x_est(3) - best_state_estimate(2));


    %target_point(1) = best_tag_estimation_x + tag_window(1) + (tag_window(2)-tag_window(1))*rand();
    %target_point(2) = best_tag_estimation_y + tag_window(1) + (tag_window(2)-tag_window(1))*rand();
%
    %[v,omega] = greedy_controller(Kp_v1,Kp_w1, target_point(1),target_point(2),robot.x_est);
%
    %x_next = robot.dynamics(v,omega);
    %dynamics_history{k,1} = x_next;
%
    %odometry_estimation = robot.odometry_step(v,omega);
    %odometry_history{k,1} = robot.x_est;

end        
