% This files contains all the parameters used while running the SLAM algorithms


Tf = 50; % Total time [s]
%Ts = 1; % Sampling time [s]
dt = 10^-3; % integration-scheme timestep


f = 867*10^6; % [Hz] frequency of the reader
c =   3*10^8; % [m/s] speed of light
lambda = c/f; % [m] wavelength

K = 2*pi/lambda;

Ns = 50; % number of precedent steps used in MHEKF

max_range = 5; % [m] maximum range of the reader
nM = ceil(max_range/(lambda/2)); % number of measurements


d = 1;   % [m] distance between the 2 wheels
R = 0.1; % [m] radius of the wheels

KL = 0.01*10^-2; % noise parameters of odometry
KR = 0.01*10^-2;

sigma_phi = 0.2; % [rad] standard deviation of the angle measurement

c1 = 0.0005; % parameter for computing weight of each instance
c2 = 0.0001; % parameter for computing weight of each instance

Kp_v1 = 0.5; % proportional parameters for PID the controller pure-pursuit
Kp_w1= 2;

Kp_v2 = 0.1; % proportional parameters for PID the controller tag-pursuit
Kp_w2= 0.2;


