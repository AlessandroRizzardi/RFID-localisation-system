
phase_history(k,1) = 0;

%{
    if isnan(best_tag_estimation_x) == true
    % it means that the robot was never in the range, moves randomly

    new_point = generateRandomPointInCircle([0,0], (x_range(2)-x_range(1))/2);

    [v,omega] = move_robot(target_point, new_point, robot.x_est, Kp_v1, Kp_w1);

else
    % the robot was already in the range of the tag and need
    % to try to go back inside --> we use the estimate we already have

    % target point needs selected randomly in the area around the best estimate of the tag we have
    % the area is defined by a cirle with a radius that is lower than the max range of the tag (tag_window)
    % and a center that is the best estimate of the tag we have

    new_point = generateRandomPointInCircle([best_tag_estimation_x,best_tag_estimation_y], tag_window);

    [v,omega] = move_robot(target_point, new_point, robot.x_est, Kp_v1, Kp_w1);
end

%}
