close all;
clear;  
clc;

addpath('Functions')
addpath('Classes')
addpath('Scripts')


% Load the configuration parameters for the algorithms
config

% Define position of the tag
tag_position = [6;6];

tag_estimation_history = [];

% Initialize the robot
robot1 = DifferentialDriveRobot([0;0;0],R,d,KR,KL,dt);
robot2 = DifferentialDriveRobot([1;1;0],R,d,KR,KL,dt);
robot3 = DifferentialDriveRobot([-2;-4;0],R,d,KR,KL,dt);

robots = [robot1,robot2, robot3];
steps = Tf/robot1.dt;

% 1st virtual target point
% random number between -5 and 5
target_point1 = generateRandomPointInCircle([0,0],(x_range(2) - x_range(1))/2);
target_point2 = generateRandomPointInCircle([0,0],(x_range(2) - x_range(1))/2);
target_point3 = generateRandomPointInCircle([0,0],(x_range(2) - x_range(1))/2);

init = [false,false,false];

init = false; % set the status of the robot if it is in the range of the RFID tag or not

for i = 1:nM
    EKF_instances1(i) = EKF();
    EKF_instances2(i) = EKF();
    EKF_instances3(i) = EKF();
end

MHEKFs = [EKF_instances1,EKF_instances2,EKF_instances3];

steps_in_range = [0,0,0];

weight_init = 1/nM;
weights_vec = weight_init*ones(nM,3);

instance_selected = [0,0,0]; 

best_tag_estimation1 = [NaN,NaN];
best_tag_estimation2 = [NaN,NaN];
best_tag_estimation3 = [NaN,NaN];

best_tag_estimations = [best_tag_estimation1,best_tag_estimation2,best_tag_estimation3];


fprintf('--------- Steps da fare: %d ---------\n\n',steps);

for k = 1:steps

    for i=1:length(robots)
        if init(i) == false && robots(i).inTagRange(tag_position,max_range) == true
        end
        
    end

    

    if mod(k,500) == 0    
        fprintf('--------- Step numero: %d ---------\n',k);
    end
end

% for l=1:nM
%     innovations_history(l,:) = EKF_instances(l).innovation_history;
% end

fprintf('           END SIMULATION\n')

%%
%final_plot

fprintf('           END ANIMATION\n')