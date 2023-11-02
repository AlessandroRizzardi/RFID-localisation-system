% NOTE
% Da mettere nella classse DifferentialDrive Robots: 
% 1) un estensione EKF, o ancora meglio creare una classe MHEKF in cui mettere tutte le istanze (altra idea: mettere il vettore delle instanze nel robot)
% 2) steps_in_range (forse meglio in EKF)
% 3) best_tag_estimation, best_state_estimate in classe robot/MHEKF
% 4) anche selected_instances fa spostato in robot in consequenza a (1)
% 5) possiamo fare a meno di go_in
% 6) possiamo passare alla classe robot tutte le histories
%
%
%
%
%
%
%
%

close all;
clear;  
clc;

addpath('Functions')
addpath('Classes')
addpath('Scripts')

% Load the configuration parameters for the algorithms
config

robot1 = DifferentialDriveRobot([0;0;0],R,d,KR,KL,dt);
robot2 = DifferentialDriveRobot([1;1;0],R,d,KR,KL,dt);


target_point1(1) = x_range(1) + (x_range(2)-x_range(1))*rand();
target_point1(2) = y_range(1) + (y_range(2)-y_range(1))*rand();

target_point2(1) = x_range(1) + (x_range(2)-x_range(1))*rand();
target_point2(2) = y_range(1) + (y_range(2)-y_range(1))*rand();




robots = [robots1;robots2];
targets = [target_point1;target_point2];



steps = Tf/robot.dt;

for k = 1:steps
    for i = 1:len(robots)
        if robots(i).init == false && robots(i).inTagRange(tag_position, max_range) == true      % check if it is the first time the phase measurement is available

            init_alghoritm           %initialize MHEKF instances
       
        elseif robots(i).init == true && robots(i).inTagRange(tag_position, max_range) == true   % check if the robot is still in range of the tag
    
            EKF_alghoritm            %performing prediction and correction
            
            weighting_alghoritm      %alghoritm that chooses an estimate between the multiple hypotesis
    
            steps_in_range = steps_in_range + 1;
    
        elseif  robots(i).inTagRange(tag_position, max_range) == false                 % check if the robot is out of range of the tag
            
            steps_in_range = 0;
    
            outOfRange_alghoritm    
    
        end
    end
end