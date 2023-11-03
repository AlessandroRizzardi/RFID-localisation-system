% robot is inside the range of the tag
init = true;
phase_measured = robot.phaseMeasured(tag_position, lambda , sigma_phi);

% Initialize nM EKF instances (l = 1,2,...,nM)
for l = 1:nM
    EKF_instances(l).EKF_init(phase_measured,l,lambda,sigma_phi,weight_init);
end

rho_est = EKF_instances(nM).x(1);
beta_est = EKF_instances(nM).x(2);

best_state_estimate = [rho_est,beta_est];

best_tag_estimation_x = robot.x_est(1) + rho_est*cos(robot.x_est(3) - beta_est);
best_tag_estimation_y = robot.x_est(2) + rho_est*sin(robot.x_est(3) - beta_est);

tag_estimation_history = [tag_estimation_history;best_tag_estimation_x,best_tag_estimation_y];


target_point = generateRandomPointInCircle([best_tag_estimation_x,best_tag_estimation_y], tag_window);

[v,omega] = greedy_controller(Kp_v1,Kp_w1, target_point(1),target_point(2),robot.x_est);
