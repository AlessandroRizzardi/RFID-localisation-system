% This files contains all the parameters used while running the SLAM algorithms


Tf = 100; % Total time [s]
Ts = 0.01; % Sampling time [s]


f = 867*10^6; % [Hz] frequency of the reader
c =   3*10^8; % [m/s] speed of light
lambda = c/f; % [m] wavelength

Ns = 50; % number of precedent steps used in MHEKF

max_range = 5; % [m] maximum range of the reader
nM = ceil(max_range/(lambda/2)); % number of measurements


L = 1;   % [m] half of the distance between the 2 wheels
R = 0.1; % [m] radius of the wheels

KL = 0.01*10^-2; % noise parameters of odometry
KR = 0.01*10^-2;

sigma_phi = 0.2; % [rad] standard deviation of the angle measurement

c1 = 0.0005; % parameter for computing weight of each instance
c2 = 0.0001; % parameter for computing weight of each instance

Kp_v = 0.01; % parameters for the controller
Kp_w = 0.01;