% Initialize the robot
robot = DifferentialDriveRobot([0;0;0],R,d,KR,KL,dt);

% Define position of the tag
tag_position = [3;3];

% 1st virtual target point
% random number between -5 and 5
target_point(1) = x_range(1) + (x_range(2)-x_range(1))*rand();
target_point(2) = y_range(1) + (y_range(2)-y_range(1))*rand();
points_vector = [];
points_vector(1,1) = target_point(1);
points_vector(1,2) = target_point(2);

init = false; % set the status of the robot if it is in the range of the RFID tag or not

for i = 1:nM
    EKF_instances(i) = EKF();
end

steps_in_range = 0;

weights_vec = zeros(nM,1);
weight_init = 1/nM;

weights_tmp = zeros(nM,1);
weights = zeros(nM,1);
weights_prev = 10^-6*ones(nM,1);

instance_selected = 0; 

go_in = true;  % flag to check if the robot is going towards the tag or not

steps = Tf/robot.dt;

best_tag_estimation_y = NaN;
best_tag_estimation_x = NaN;