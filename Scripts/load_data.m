
if ~exist('ProcessedData', 'dir')
    mkdir('ProcessedData')
end


%   ___      _                      _                    _       _        
%  / _ \  __| | ___  _ __ ___   ___| |_ _ __ _   _    __| | __ _| |_ __ _ 
% | | | |/ _` |/ _ \| '_ ` _ \ / _ \ __| '__| | | |  / _` |/ _` | __/ _` |
% | |_| | (_| | (_) | | | | | |  __/ |_| |  | |_| | | (_| | (_| | || (_| |
%  \___/ \__,_|\___/|_| |_| |_|\___|\__|_|   \__, |  \__,_|\__,_|\__\__,_|
%                                            |___/                        
if load_precomputed_data && exist('ProcessedData/odometries.mat', 'file')

    fprintf('Loading data from ProcessedData/odometries.mat... ');
    load('ProcessedData/odometries.mat');
    fprintf('Done!\n');

else % must build the file
    
    fprintf('Generating odometry data from Data/simul_ODO.txt\n');
    odometry_data   = readmatrix(select_odometries);
    N_odometries    = size(odometry_data, 1);
    odometries      = cell(1, N_odometries);
    fprintf('Number of odometry entries: %d\n', N_odometries);

    for i = 1:N_odometries
        odometries{i} = Odometry(odometry_data(i, :));
    end
    fprintf('Saving into ProcessedData/odometries.mat... ');
    save('ProcessedData/odometries.mat', 'odometries', 'N_odometries');
    fprintf('Done!\n');

end
clearvars odometry_data i


%  _____ _                     
% |_   _(_)_ __ ___   ___  ___ 
%   | | | | '_ ` _ \ / _ \/ __|
%   | | | | | | | | |  __/\__ \
%   |_| |_|_| |_| |_|\___||___/
%          
if load_precomputed_data && exist('ProcessedData/times.mat', 'file')  
    
    fprintf('Loading time vectors from ProcessedData/times.mat... ');
    load('ProcessedData/times.mat');
    fprintf('Done!\n');

else % must build the file
    
    fprintf('and Data/simul_ODO_times.txt\n');
    odometries_times    = readmatrix(select_odo_times);
    dt_odometries       = mean(odometries_times(2:end) - odometries_times(1:end-1));

    odometries_times = 0:dt_odometries:(N_odometries-1)*dt_odometries;


    fprintf('Saving into ProcessedData/times.mat... ');
    save('ProcessedData/times.mat', 'odometries_times', 'dt_odometries');
    fprintf('Done!\n');

end

clearvars load_precomputed_data