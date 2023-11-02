close all;
clear;  
clc;

addpath('Functions')
addpath('Classes')
addpath('Scripts')


% Load the configuration parameters for the algorithms
config

initialization_vars

for k = 1:steps

    if init == false && robot.inTagRange(tag_position, max_range) == true      % check if it is the first time the phase measurement is available

        init_alghoritm           %initialize MHEKF instances
   
    elseif init == true && robot.inTagRange(tag_position, max_range) == true   % check if the robot is still in range of the tag

        EKF_alghoritm            %performing prediction and correction
        
        weighting_alghoritm      %alghoritm that chooses an estimate between the multiple hypotesis

        steps_in_range = steps_in_range + 1;

    elseif  robot.inTagRange(tag_position, max_range) == false                 % check if the robot is out of range of the tag
        
        steps_in_range = 0;

        outOfRange_alghoritm    

    end

    if mod(k,1000) == 0    
        fprintf('--------- Step numero: %d\n ---------',k);
    end
end


final_plot



