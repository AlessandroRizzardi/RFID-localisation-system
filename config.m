% This files contains all the parameters used while running the SLAM algorithms


Tf = 120; % Total time [s]
%Ts = 1; % Sampling time [s]
dt = 10^-2; % integration-scheme timestep


f = 867*10^6; % [Hz] frequency of the reader
c =   3*10^8; % [m/s] speed of light
lambda = c/f; % [m] wavelength

K = 2*pi/lambda;

Ns = 50; % number of precedent steps used in MHEKF

max_range = 2; % [m] maximum range of the reader
nM = ceil(max_range/(lambda/2)); % number of measurements
%nM = 10;

d = 0.5;   % [m] distance between the 2 wheels
R = 0.2; % [m] radius of the wheels

KL = 0.01*10^-2; % noise parameters of odometry
KR = 0.01*10^-2;

sigma_phi = 0.2; % [rad] standard deviation of the angle measurement

c1 = 0.0005; % parameter for computing weight of each instance
c2 = 0.0001; % parameter for computing weight of each instance

Kp_v1 = 0.8; % proportional parameters for PID the controller pure-pursuit
Kp_w1= 2;

Kp_v2 = 0.1; % proportional parameters for PID the controller tag-pursuit
Kp_w2= 0.2;


x_range = [-5 5]; % [m] range of the map
y_range = [-5 5]; % [m] range of the map
radius_map = (x_range(2) - x_range(1))*sqrt(2)/2; % [m] radius of the map

method_paper = false;

tag_window = max_range;

weight_init = 1/nM;