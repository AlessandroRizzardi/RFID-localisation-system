% robot is inside the range of the tag
tag_found_flag = true;
tag_found_position = [robots(i).x_est(1), robots(i).x_est(2)];

robots(i).init_flag = true;
%phase_measured = robots(i).phaseMeasured(tag_position, lambda , sigma_phi);

% Initialize nM EKF instances (l = 1,2,...,nM)
for l = 1:nM
    MHEKFs(i,l).EKF_init(robots(i).phase_measured,l,lambda,sigma_phi,weight_init, robots(i).x_est, robots(i).covariance_matrix);
    
    % saving the state
    MHEKFs(i,l).state_history = [MHEKFs(i,l).state_history; MHEKFs(i,l).x(1), MHEKFs(i,l).x(2)];
    
end

rho_est = MHEKFs(i,nM).x(1);
beta_est = MHEKFs(i,nM).x(2);

%best_state_estimate = [rho_est,beta_est];

robots(i).best_tag_estimation(2) = robots(i).x_est(2) + rho_est*sin(robots(i).x_est(3) - beta_est);
robots(i).best_tag_estimation(1) = robots(i).x_est(1) + rho_est*cos(robots(i).x_est(3) - beta_est);

robots(i).tag_estimation_history = [robots(i).tag_estimation_history; robots(i).best_tag_estimation(1), robots(i).best_tag_estimation(2)];


targets(i,:) = generateRandomPointInCircle(robots(i).best_tag_estimation, max_range);
target_point = targets(i,:);
[v,omega] = greedy_controller(Kp_v1, Kp_w1, target_point(1),target_point(2), robots(i).x_est);  
