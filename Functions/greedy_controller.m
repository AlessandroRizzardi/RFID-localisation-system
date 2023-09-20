function [v,dtheta] = greedy_controller(Kp_v,Kp_w, x_target, y_target, odometry_state)
    x_odometry = odometry_state(1);
    y_odometry = odometry_state(2);
    theta_odometry = odometry_state(3);

    theta_desired = atan2(y_target - y_odometry, x_target - x_odometry); 

    
    error_heading = atan2(sin(theta_desired-theta_odometry),cos(theta_desired-theta_odometry));
    dtheta = Kp_w*error_heading;
    
    dist = sqrt((y_target - y_odometry)^2 + (x_target - x_odometry)^2); 
    v = Kp_v * dist;
end