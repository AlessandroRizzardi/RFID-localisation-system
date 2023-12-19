tag_found_flag = false;
tag_flag_vector = zeros(nRobots,1);

% Initialize the swarm
robots = [];
for i=1:nRobots
    random_initial_position = generateRandomPointOnCircumferences([0,0], radius_map); 
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
% time vector
t = 0:dt:Tf;

% 1st virtual target points
targets = [];

for i=1:nRobots
    target = generateRandomPointInCircle([tag_position(1),tag_position(2)],1); % actually set to go towards the target, change for real simulation
    targets = [targets; target];
end

last_nonNaN_estimation = NaN;

%fprintf('--------- Steps da fare: %d ---------\n\n',steps);