%phase_measured = robots(i).phaseMeasured(tag_position, lambda , sigma_phi);
% phase_history(k,1) = phase_measured; #TODO: check if this part is used somewhere different from euristic weighing algorithm

% Prediction and Correction EKF
for l = 1:nM
    MHEKFs(i,l).EKF_predict(robots(i).odometry_estimation, d);
    MHEKFs(i,l).EKF_correct(K, sigma_phi, robots(i).phase_measured);
end


for l = 1:nM
    robots(i).weights_vec(l) = MHEKFs(i,l).weight;
end

for l = 1:nM
    MHEKFs(i,l).weight = MHEKFs(i,l).weight/sum(robots(i).weights_vec);
end

for l = 1:nM
    robots(i).weights_vec(l) = MHEKFs(i,l).weight;
end

% Correction of non-positive range estimation and range estimation too low
for l = 1:nM
    if MHEKFs(i,l).x(1) < 10^-6
        number_states = length( MHEKFs(i,l).state_history);
        MHEKFs(i,l).x(1) = max([abs(MHEKFs(i,l).state_history(max(1,number_states-5),1)),10^-6]); % I choose the range of 5 steps before
        MHEKFs(i,l).x(2) = MHEKFs(i,l).state_history(max(1,number_states-5),2) + pi;
    end
    MHEKFs(i,l).x(2) = atan2(sin(MHEKFs(i,l).x(2)),cos(MHEKFs(i,l).x(2)));
end


% saving the state
for l = 1:nM
    MHEKFs(i,l).state_history = [MHEKFs(i,l).state_history; MHEKFs(i,l).x(1), MHEKFs(i,l).x(2)];
end

% Weighing Step
[max_value,instance_selected] = max(robots(i).weights_vec);
robots(i).instance_selected = instance_selected;

rho_est = MHEKFs(i, instance_selected).x(1);
beta_est = MHEKFs(i, instance_selected).x(2);

%best_state_estimate = [rho_est,beta_est];

robots(i).best_tag_estimation(1,1) = robots(i).x_est(1) + rho_est*cos(robots(i).x_est(3) - beta_est);
robots(i).best_tag_estimation(2,1) = robots(i).x_est(2) + rho_est*sin(robots(i).x_est(3) - beta_est);

robots(i).tag_estimation_history{k,1} = robots(i).best_tag_estimation;

if robots(i).distanceFromPoint(targets(i,:)) <= 0.8
    targets(i,:) = generateRandomPointInCircle(robots(i).best_tag_estimation, 0.5*max_range);
end

target_point = targets(i,:);
[v,omega] = greedy_controller(Kp_v1, Kp_w1, target_point(1),target_point(2), robots(i).x_est); 

