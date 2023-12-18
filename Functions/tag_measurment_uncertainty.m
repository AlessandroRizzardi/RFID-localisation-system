function[R] = tag_measurment_uncertainty(P, robot)

    rho_est = robot.best_tag_estimation(1);
    beta_est = robot.best_tag_estimation(2);

    % jacobian of the position of the tag wrt the state (rho,beta,x,y,theta)
    robot.J_h = [cos(robot.x_est(3) - beta_est), rho_est*sin(robot.x_est(3) - beta_est), 1, 0, -rho_est*sin(robot.x_est(3) - beta_est);...
                     sin(robot.x_est(3) - beta_est), -rho_est*cos(robot.x_est(3) - beta_est), 0, 1,  rho_est*cos(robot.x_est(3) - beta_est)];

    R = robot.J_h * P * robot.J_h';
    
end