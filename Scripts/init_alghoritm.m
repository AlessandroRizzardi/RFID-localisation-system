% robot is inside the range of the tag
tag_found_flag = true;
tag_found_position = robots(i).x_est;

robots(i).init = true;
phase_measured = robots(i).phaseMeasured(tag_position, lambda , sigma_phi);

% Initialize nM EKF instances (l = 1,2,...,nM)
for l = 1:nM
    MHEKFs(i,l).EKF_init(phase_measured,l,lambda,sigma_phi,weight_init);
    
    % saving the state
    MHEKFs(i,l).state_history = [MHEKFs(i,l).state_history; MHEKFs(i,l).x];
    
end

rho_est = MHEKFs(i,nM).x(1);
beta_est = MHEKFs(i,nM).x(2);

%best_state_estimate = [rho_est,beta_est];

robots(i).best_tag_estimation(1) = robot(i).x_est(1) + rho_est*cos(robot(i).x_est(3) - beta_est);
robots(i).best_tag_estimation(2) = robot(i).x_est(2) + rho_est*sin(robot(i).x_est(3) - beta_est);

robots(i).tag_estimation_history = [robot(i).tag_estimation_history; robots(i).best_tag_estimation(1), robots(i).best_tag_estimation(2)];


targets(i,:) = robots(i).best_tag_estimation;
target_point = targets(i,:);
[v,omega] = greedy_controller(Kp_v1, Kp_w1, target_point(1),target_point(2), robots(i).x_est);  


x_next = robots(i).dynamics(v,omega);
robots(i).dynamics_history{k,1} = x_next;

odometry_estimation = robots(i).odometry_step(v,omega);
robots(i).odometry_history{k,1} = robots(i).x_est;