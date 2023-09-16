function [v,omega] = greedy_controller(Kp_v, x_target, y_target, odometry_state,Ts)
    x_odometry = odometry_state(1);
    y_odometry = odometry_state(2);

    theta_target = atan2(y_target - y_odometry, x_target - x_odometry); 

    if abs(theta_target - odometry_state(3)) < pi/6
        omega = 0;
    else
        omega = theta_target;
    end
    
    dist = sqrt((y_target - y_odometry)^2 + (x_target - x_odometry)^2); 
    v = 0.1*Ts;
end