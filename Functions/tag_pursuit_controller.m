function [v,w] = tag_pursuit_controller(Kp_v, Kp_w, rho_est,beta_est)
    v = Kp_v * rho_est * cos(beta_est);
    w = -Kp_w * beta_est;
end