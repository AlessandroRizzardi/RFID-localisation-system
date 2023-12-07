function[sigma_x, sigma_y] = tag_measurment_uncertainty(state, P)

    x = state(1);
    y = state(2);
    theta = state(3);
    rho = state(4);
    beta = state(5);


    dh1_dx = 1;
    dh1_dy = 0;
    dh1_dthetha = - rho*sin(theta-beta);
    dh1_drho = cos(theta-beta);
    dh1_dbeta = rho*sin(theta-beta);

    dh2_dx = 0;
    dh2_dy = 1;
    dh2_dthetha = rho*cos(theta-beta);
    dh2_drho = sin(theta-beta);
    dh2_dbeta = -rho*cos(theta-beta);

    dh1_ds = [dh1_dx,dh1_dy,dh1_dthetha,dh1_drho,dh1_dbeta];
    dh2_ds = [dh2_dx,dh2_dy,dh2_dthetha,dh2_drho,dh2_dbeta];

    sigma_x = 0;
    sigma_y = 0;

    % TODO: implmenting efficient algorithm to compute propagation of uncertainty
    
end