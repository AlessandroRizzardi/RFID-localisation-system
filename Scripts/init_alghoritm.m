init = true;
phase_measured = robot.phaseMeasured(tag_position, lambda , sigma_phi);
phase_history(k,1) = phase_measured;

% Initialize nM EKF instances (l = 1,2,...,nM)
for l = 1:nM
    EKF_instances(l).EKF_init(phase_measured,l,lambda,sigma_phi,weight_init);

    % Save the state history of each EKF instance
    EKF_instances(l).state_history{k,1} = EKF_instances(l).x;

    if isnan(EKF_instances(l).state_history{k,1}) == true
        fprintf('Error state history\n');
    end

end

rho_est = EKF_instances(nM).x(1);
beta_est = EKF_instances(nM).x(2);
best_state_estimate = [rho_est;beta_est];

best_tag_estimation_x = robot.x_est(1) + best_state_estimate(1)*cos(robot.x_est(3) - best_state_estimate(2));
best_tag_estimation_y = robot.x_est(2) + best_state_estimate(1)*sin(robot.x_est(3) - best_state_estimate(2));

display('Enter in tag-range at time step:')
display(k)

target_point(1) = best_tag_estimation_x + tag_window(1) + (tag_window(2)-tag_window(1))*rand();
target_point(2) = best_tag_estimation_y + tag_window(1) + (tag_window(2)-tag_window(1))*rand();

[v,omega] = greedy_controller(Kp_v1,Kp_w1, target_point(1),target_point(2),robot.x_est);

x_next = robot.dynamics(v,omega);
dynamics_history{k,1} = x_next;

odometry_estimation = robot.odometry_step(v,omega);

odometry_history{k,1} = robot.x_est;