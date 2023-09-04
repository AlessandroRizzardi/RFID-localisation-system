% This files contains all the parameters used while running the SLAM algorithms


% 
%  __  __       _          ___     _                    _       _       _        
% |  \/  | __ _(_)_ __    ( _ )   | |    ___   __ _  __| |   __| | __ _| |_ __ _ 
% | |\/| |/ _` | | '_ \   / _ \/\ | |   / _ \ / _` |/ _` |  / _` |/ _` | __/ _` |
% | |  | | (_| | | | | | | (_>  < | |__| (_) | (_| | (_| | | (_| | (_| | || (_| |
% |_|  |_|\__,_|_|_| |_|  \___/\/ |_____\___/ \__,_|\__,_|  \__,_|\__,_|\__\__,_|
%                                                                                

load_precomputed_data = true; % Use precomputed features                  
GT = true; % Compare the grand truth

select_odometries = 'simul_ODO_1.txt';
select_odo_times = 'simul_ODO_times_1.txt';
select_GT = 'simul_GT_1.txt';
save_datas = true;

Ts = 0.2; % Sampling time [s]


%   _____ _     __ 
%  | ____| | __/ _|
%  |  _| | |/ / |_ 
%  | |___|   <|  _|
%  |_____|_|\_\_|  
%   
min_distance_features = 0.8; % If 2 features are closer then they are collapsed into only one
plot_figure = false; % to plot the incremental map

%  ____       _           _   
% |  _ \ ___ | |__   ___ | |_ 
% | |_) / _ \| '_ \ / _ \| __|
% |  _ < (_) | |_) | (_) | |_ 
% |_| \_\___/|_.__/ \___/ \__|
%                             

f = 865.7*10^6; % [Hz] frequency of the reader
c = 3*10^8; % [m/s] speed of light
lambda = c/f; % [m] wavelength


%   ___      _                      _              
% / _ \  __| | ___  _ __ ___   ___| |_ _ __ _   _ 
% | | | |/ _` |/ _ \| '_ ` _ \ / _ \ __| '__| | | |
% | |_| | (_| | (_) | | | | | |  __/ |_| |  | |_| |
%  \___/ \__,_|\___/|_| |_| |_|\___|\__|_|   \__, |
%                                           |___/ 
%

               

