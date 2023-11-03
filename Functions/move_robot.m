function [v,omega] = move_robot(target_point, new_point, pos_robot, Kp_v1, Kp_w1)

    % distance = sqrt((pos_robot(1) - target_point(1))^2 + (pos_robot(2) - target_point(2))^2);
    % 
    % if distance < 0.001
    %     % the robot is close to the target point, we need to select a new one
    % 
    % end
    target_point = new_point;

    [v,omega] = greedy_controller(Kp_v1, Kp_w1, target_point(1),target_point(2),pos_robot);

end