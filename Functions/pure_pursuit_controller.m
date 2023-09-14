function [v,omega] = pure_pursuit_controller(Kp_w, Kp_v, x_target, y_target, odometry_state)
    x_odometry = odometry_state(1);
    y_odometry = odometry_state(2);
    theta_odometry = odometry_state(3);

    theta_target = atan2(y_target - y_odometry, x_target - x_odometry);
    omega = Kp_w * (theta_target - theta_odometry);
    
    dist = sqrt((y_target - y_odometry)^2 + (x_target - x_odometry)^2); 
    v = Kp_v * dist;
    
end