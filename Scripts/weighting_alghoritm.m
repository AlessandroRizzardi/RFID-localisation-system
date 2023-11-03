
% Weighing Step

[max_value,instance_selected] = max(weights_vec);

rho_est = EKF_instances(instance_selected).x(1);
beta_est = EKF_instances(instance_selected).x(2);

best_state_estimate = [rho_est,beta_est];

best_tag_estimation_x = robot.x_est(1) + rho_est*cos(robot.x_est(3) - beta_est);
best_tag_estimation_y = robot.x_est(2) + rho_est*sin(robot.x_est(3) - beta_est);

tag_estimation_history = [tag_estimation_history;best_tag_estimation_x,best_tag_estimation_y];
