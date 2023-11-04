phase_measured = robot.phaseMeasured(tag_position, lambda , sigma_phi);
phase_history(k,1) = phase_measured;

% Prediction and Correction EKF
for l = 1:nM
    EKF_instances(l).EKF_predict(odometry_estimation, d);
    EKF_instances(l).EKF_correct(K, sigma_phi, phase_measured);
    
    weights_vec(l) = EKF_instances(l).weight;
end

weights_vec = weights_vec/sum(weights_vec); % Normalization of weights

% Correction of non-positive range estimation and range estimation too low
for l = 1:nM
    if EKF_instances(l).x(1) < 10^-6
        number_states = length( EKF_instances(l).state_history);
        EKF_instances(l).x(1) = max([abs(EKF_instances(l).state_history(max(1,number_states-5),1)),10^-6]); % I choose the range of 5 steps before
        EKF_instances(l).x(2) = EKF_instances(l).state_history(max(1,number_states-5),2) + pi;
    end
    EKF_instances(l).x(2) = atan2(sin(EKF_instances(l).x(2)),cos(EKF_instances(l).x(2)));
end


% saving the state
for l = 1:nM
    EKF_instances(l).state_history = [EKF_instances(l).state_history; EKF_instances(l).x];
end