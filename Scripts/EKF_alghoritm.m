phase_measured = robot.phaseMeasured(tag_position, lambda , sigma_phi);
phase_history(k,1) = phase_measured;

% Prediction and Correction EKF
for l = 1:nM
    EKF_instances(l).EKF_predict(odometry_estimation, d);
    EKF_instances(l).EKF_correct(K, sigma_phi, phase_measured);
end


for l = 1:nM
    weights_vec(l) = EKF_instances(l).weight;
end

for l = 1:nM
    EKF_instances(l).weight = EKF_instances(l).weight/sum(weights_vec);
end

for l = 1:nM
    weights_vec(l) = EKF_instances(l).weight;
end

weights_sum(k) = sum(weights_vec);

weights_history(:,k) = weights_vec;

% Correction of non-positive range estimation and range estimation too low
for l = 1:nM
    if EKF_instances(l).x(1) < 10^-6
        number_states = length( EKF_instances(l).state_history);
        EKF_instances(l).x(1) = max([abs(EKF_instances(l).state_history(max(1,number_states-5),1)),10^-6]); % I choose the range of 5 steps before
        EKF_instances(l).x(2) = EKF_instances(l).state_history(max(1,number_states-5),2) + pi;
    end
    EKF_instances(l).x(2) = atan2(sin(EKF_instances(l).x(2)),cos(EKF_instances(l).x(2)));
end


% saving the state
for l = 1:nM
    EKF_instances(l).state_history = [EKF_instances(l).state_history; EKF_instances(l).x];
end

% Weighing Step
[max_value,instance_selected] = max(weights_vec);

rho_est = EKF_instances(instance_selected).x(1);
beta_est = EKF_instances(instance_selected).x(2);

best_state_estimate = [rho_est,beta_est];

best_tag_estimation_x = robot.x_est(1) + rho_est*cos(robot.x_est(3) - beta_est);
best_tag_estimation_y = robot.x_est(2) + rho_est*sin(robot.x_est(3) - beta_est);

tag_estimation_history = [tag_estimation_history;best_tag_estimation_x,best_tag_estimation_y];
