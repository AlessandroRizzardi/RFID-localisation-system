% Define position of the tag
tag_position = [0;0];

tag_estimation_history = [];

% Initialize the robot
initial_position = generateRandomPointInCircle([0,0], max_range);
robot = DifferentialDriveRobot([initial_position(1);initial_position(2);0],R,d,KR,KL,dt);
steps = Tf/robot.dt;

% 1st virtual target point
% random number between -5 and 5
target_point = generateRandomPointInCircle([0,0],max_range);

init = false; % set the status of the robot if it is in the range of the RFID tag or not

for i = 1:nM
    EKF_instances(i) = EKF();
end

steps_in_range = 0;

weight_init = 1/nM;
weights_vec = weight_init*ones(nM,1);

weights_history = zeros(nM,steps);
weights_history(:,1) = weights_vec;

innovations_history = zeros(nM,steps-1);

instance_selected = 0; 

go_in = true;  % flag to check if the robot is going towards the tag or not

weights_sum = zeros(1,steps);

best_tag_estimation_y = NaN;
best_tag_estimation_x = NaN;

