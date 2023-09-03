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

% initializating the 4 RFID tags, the position is random
tag1 = Tag([(10+5).*rand(1,1) - 5; (10+5).*rand(1,1) - 5;])
tag2 = Tag([(10+5).*rand(1,1) - 5; (10+5).*rand(1,1) - 5;])
tag3 = Tag([(10+5).*rand(1,1) - 5; (10+5).*rand(1,1) - 5;])
tag4 = Tag([(10+5).*rand(1,1) - 5; (10+5).*rand(1,1) - 5;])

num_tags = 4;
RFIDsystem = RFIDreadings([tag1; tag2; tag3; tag4], num_tags);

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
  %robot_old = robot;

  robot.P = P_est;

  % KF update -----------------------------------------------------------
  fprintf('Update...');

  robot_act = RobotReader();

  if k == 1
    robot_act.x = [gt(1,2); gt(1,3); gt(1,4); gt(1,2); gt(1,3); gt(1,4)];
  else
    robot_act.x = [gt(k,2); gt(k,3); gt(k,4); gt(k-1,2); gt(k-1,3); gt(k-1,4)];
  end

  [h,H] = robot.measurement_function(RFIDsystem.tags_vector, RFIDsystem.num_tags);

  [h_actual, ~] = robot_act.measurement_function(RFIDsystem.tags_vector, RFIDsystem.num_tags);
  
  K = P_est*H'*inv(H*P_est*H' + RFIDsystem.R);
  x_est = x_est + K*(h_actual - h);
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
