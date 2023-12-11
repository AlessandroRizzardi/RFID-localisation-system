tag_found_flag = false;
tag_found_position = [0,0];
tag_flag_vector = zeros(nRobots,1);

% Initialize the swarm
robots = [];
for i=1:nRobots
    random_initial_position = generateRandomPointInCircle([0,0], radius_map); 
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
    target = generateRandomPointInCircle([3,3],2); % actually set to go towards the target, change for real simulation
    targets = [targets; target];
end

fprintf('--------- Steps da fare: %d ---------\n\n',steps);