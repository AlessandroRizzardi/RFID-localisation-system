for l=1:nM
    % Save the state history of each EKF instance --> void because the robot is out of rangex\
    EKF_instances(l).state_history{k,1} = [];

end

phase_history(k,1) = 0;

if isnan(best_tag_estimation_x)
    % it means that the robot was already in the range of the tag and need
    % to try to go back inside --> we use the estimate we already have

    x_target = target_point(1);
    y_target = target_point(2);
    distance = sqrt((robot.x_est(1) - x_target)^2 + (robot.x_est(2) - y_target)^2);

    if distance < 0.5
        target_point(1) = x_range(1) + (x_range(2)-x_range(1))*rand();
        target_point(2) = y_range(1) + (y_range(2)-y_range(1))*rand();  
        points_vector(end+1,:) = [target_point(1),target_point(2)];
        [v,dtheta] = greedy_controller(Kp_v1, Kp_w1, target_point(1),target_point(2),robot.x_est);
    else
        [v,dtheta] = greedy_controller(Kp_v1,Kp_w1, target_point(1),target_point(2),robot.x_est);
    end

else
    % the robot was never in the range, moves randomly

    target_point(1) = best_tag_estimation_x + tag_window(1) + (tag_window(2)-tag_window(1))*rand();
    target_point(2) = best_tag_estimation_y + tag_window(1) + (tag_window(2)-tag_window(1))*rand();

    [v,omega] = greedy_controller(Kp_v1,Kp_w1, target_point(1),target_point(2),robot.x_est);
end

x_next = robot.dynamics(v,dtheta);
dynamics_history{k,1} = x_next;

odometry_estimation = robot.odometry_step(v,dtheta);
odometry_history{k,1} = robot.x_est;
