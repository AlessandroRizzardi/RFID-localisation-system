close all;
clear;  
clc;

addpath('Functions')
addpath('Classes')
addpath('Scripts')


% Load the configuration parameters for the algorithms
config

%%%%%%%%%%  SETTINGS    %%%%%%%%%%%%%
% Define position of the tag
tag_position = [3;3];
nRobots = 2;
%%%%%%%%%%  END SETTINGS    %%%%%%%%%%%%%

tag_found_flag = false;
tag_found_position = [0,0];

% Initialize the swarm
robots = [];
for i=1:nRobots
    random_initial_position = generateRandomPointInCircle([0,0], (x_range(2) - x_range(1))/2); 
    robot = DifferentialDriveRobot([random_initial_position(1); random_initial_position(2); 0],R,d,KR,KL,dt,nM);
    robots = [robots, robot];
end

% 3x10 matrix
for i=1:nRobots
    for l=1:nM
        MHEKFs(i,l) = EKF();
    end
end

steps = Tf/dt;

% 1st virtual target points
targets = [];
for i=1:nRobots
    target = generateRandomPointInCircle([3,3],2); % actually set to go towards the target, change for real simulation
    targets = [targets; target];
end


fprintf('--------- Steps da fare: %d ---------\n\n',steps);

for k = 1:steps

    phases = [];

    for i=1:length(robots)
        if robots(i).inTagRange(tag_position, max_range) == true 
            robots(i).phaseMeasured(tag_position, lambda , sigma_phi);
            phases = [phases; robots(i).phase_measured];
        end
    end

    for i=1:length(robots)
        
        if robots(i).init_flag == false && robots(i).inTagRange(tag_position,max_range) == true
            
            fprintf('Tag found by robot %d \n', i);
           
            init_alghoritm
        
        elseif robots(i).init_flag && robots(i).inTagRange(tag_position, max_range) == true

            EKF_alghoritm

            robots(i).steps_in_range = robots(i).steps_in_range + 1;

        elseif robots(i).inTagRange(tag_position, max_range) == false   % check if the robot is out of range of the tag
        
            robots(i).steps_in_range = 0;
    
            outOfRange_alghoritm  
        end

        x_next = robots(i).dynamics(v,omega);
        robots(i).dynamics_history{k,1} = x_next;

        robots(i).odometry_estimation = robots(i).odometry_step(v,omega);
        robots(i).odometry_history{k,1} = robots(i).x_est;

        robots(i).covariance_update();
        
    end

    if mod(k,500) == 0    
        fprintf('--------- Step numero: %d ---------\n',k);
    end
end

fprintf('           END SIMULATION\n')
%%

ANIMATION = false;
DRAW = true;

final_plot

fprintf('           END ANIMATION\n')