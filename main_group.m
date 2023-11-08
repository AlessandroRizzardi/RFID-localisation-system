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
tag_found_flag = false;
tag_found_position = [0,0];

nRobots = 3;

% Initialize the robot
robot1 = DifferentialDriveRobot([0;0;0],R,d,KR,KL,dt,nM);
robot2 = DifferentialDriveRobot([1;1;0],R,d,KR,KL,dt,nM);
robot3 = DifferentialDriveRobot([-2;-4;0],R,d,KR,KL,dt,nM);
robots = [robot1,robot2, robot3];

% 3x10 matrix
for i=1:nRobots
    for l=1:nM
        MHEKFs(i,l) = EKF();
    end
end

steps = Tf/robot1.dt;

% 1st virtual target point
% random number between -5 and 5
target_point1 = generateRandomPointInCircle([0,0],(x_range(2) - x_range(1))/2);
target_point2 = generateRandomPointInCircle([0,0],(x_range(2) - x_range(1))/2);
target_point3 = generateRandomPointInCircle([0,0],(x_range(2) - x_range(1))/2);

targets = [target_point1, target_point2, target_point3];

fprintf('--------- Steps da fare: %d ---------\n\n',steps);

for k = 1:steps

    for i=1:length(robots)
        if robots(i).init_flag == false && robots(i).inTagRange(tag_position,max_range) == true
            
            init_alghoritm
        
        elseif init == true && robot.inTagRange(tag_position, max_range) == true

            EKF_alghoritm

            robots(i) = robots(i).steps_in_range + 1;

        elseif robot.inTagRange(tag_position, max_range) == false   % check if the robot is out of range of the tag
        
            robots(i).steps_in_range = 0;
    
            outOfRange_alghoritm  
        end
        
    end

    if mod(k,500) == 0    
        fprintf('--------- Step numero: %d ---------\n',k);
    end
end


fprintf('           END SIMULATION\n')

%%
final_plot

fprintf('           END ANIMATION\n')