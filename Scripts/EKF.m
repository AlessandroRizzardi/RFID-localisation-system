% The model of the analized problem is:
%     
%     x(k+1) = fk(x(k),u(k),nu(k)) -> nu is the noise in the model
%     z(k) = hk(x(k),epsilon(k)) -> epsilon is the noise on the measures
% 
% The Ekf is divided into steps: 
%     - predicion:
%         x_est(k+1) = fk(x_est(k), u(k))
%         P_est(k+1) = A(k)P_est(k)A(k)' + G(k)Q(k)G(k)'
% 
%         where:
%         A(k) is the jacobian of fk(x,u,nu) w.r.t the state e and evaluated in the x_est(k) and u(k) with nu = 0
%         G(k) is the jacobian of fk(x,u,nu) w.r.t nu e and evaluated in the x_est(k) and u(k) with nu = 0
%      
%      - update:
%         S(k+1) = H(k+1)P_est(k+1)H(k+1)' + R(k+1)
%         W(k+1) = P_est(k+1)H(k+1)'/S(k+1)
%         x_est(k+1) = x_est(k+1) + W(k+1)(z(k+1) - hk+1(x_est(k+1)))
%         P_est(k+1) = (I - W(k+1)H(k+1))P_est(k+1)
% 
%         where:
%         H(k+1) is the jacobian of hk+1(x,epsilon) w.r.t the state e and evaluated in the x_est(k+1) and epsilon = 0

% state space of the filter is represented by:
% x = [x y theta x_old y_old theta_old]
x_est = zeros(6,1);
P_est = zeros(6,6);

robot = RobotReader();

% the covariance matrix expressing the vehicle initial location uncertainty is P0 = diag(0.12, 0.12, 0.12) for the first upper matrix
% and P0 = diag(0, 0, 0) for the second lower matrix
robot.P = [diag([0.12, 0.12, 0.12]), zeros(3,3); zeros(3,3), diag([0, 0, 0])];
P_est = robot.P;

% the map we are considering is about on the x-axis from -5 to 10 and on the y-axis from -2 to 10

% initializating 30 RFID tags, the position is random
%tag1 = Tag([-3; 9]);
%tag2 = Tag([8; 9]);
%tag3 = Tag([-3; -2]);
%tag4 = Tag([8; -2]);

num_tags = 5;
for i = 1:num_tags
  tags(i) = Tag([(10+5).*rand(1,1) - 5; (10+5).*rand(1,1) - 5;]);
end

RFIDsystem = RFIDreadings(tags, num_tags);

T_limit = N_odometries;

robot_old = RobotReader();

disp('Starting the cycle')
% Temporal cycle
for k = 1:T_limit

  fprintf('================================> Iteration %6d <================================\n', k);

  A = robot.JF_x(odometries{1,k});
  B = robot.JF_n();
  Q = odometries{k}.Q;
  
  
  % KF Prediction ------------------------------------------------------
  fprintf('Prediction...');
  x_est(1:3) = robot.update_state_robot(odometries{k});

  P_est = A*P_est*A' + B*Q*B';
  fprintf('Done!\n');

  % update state space
  x_est(4:6) = robot.x(4:6);

  robot.P = P_est;

  % KF update -----------------------------------------------------------
  fprintf('Update...');

  h_meas = zeros(num_tags, 1);
  for i = 1:num_tags
    tag = RFIDsystem.tags_vector(i);

    if k == 1
      d_act = tag.compute_distance([gt(1,2); gt(1,3)]);
      d_old = tag.compute_distance([gt(1,2); gt(1,3)]);
    else
      d_act = tag.compute_distance([gt(k,2); gt(k,3)]);
      d_old = tag.compute_distance([gt(k-1,2); gt(k-1,3)]);
    end

    distance_meas_act = distance_measured(d_act,lambda);
    distance_meas_old = distance_measured(d_old,lambda);

    h_meas(i) = distance_meas_act - distance_meas_old;
  end

  [h,H] = robot.measurement_function(RFIDsystem.tags_vector, RFIDsystem.num_tags);

  K = P_est*H'*inv(H*P_est*H' + RFIDsystem.R);
  x_est = x_est + K*(h_meas - h);
  P_est = (eye(6) - K*H)*P_est;


  fprintf('Done!\n');
  
  check_covariance_matrix(P_est, 'After update')  

  pos_robot{k,1} = x_est(1:3);
  cov_robot{k,1} = P_est;

end


function check_covariance_matrix(P, text)
  if ~all(eig(P) >= 0)
    if nargin == 2
      fprintf('\n\nCOVARIANCE ERROR: %s\n\n', text);
    end
    error('The covariance matrix of the observation is not positive definite');
  end
end

function plotErrorEllipse(mu, Sigma, p, color)

s = -2 * log(1 - p);

[V, D] = eig(Sigma * s);

t = linspace(0, 2 * pi);
a = (V * sqrt(D)) * [cos(t(:))'; sin(t(:))'];

plot(a(1, :) + mu(1), a(2, :) + mu(2),color);
end
