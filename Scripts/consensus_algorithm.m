%number of consensus protocol messages exchanged by the nodes in the network
m = 50;

% Initialisation
F = cell(nRobots, 1);
a = cell(nRobots, 1);

for i = 1:nRobots
    zi = robots(i).best_tag_estimation;
    Hi = robots(i).J_h;
    F{i} = Hi'*inv(robots(i).covariance_matrix)*Hi;
    a{i} = Hi'*inv(robots(i).covariance_matrix)*zi;
end

for k = 1:m

    % Topology matrix: 1 if there is a link between the nodes, 0 otherwise
    % We assume that when the robots are in range they can communicate
    A = ones(nRobots) - eye(nRobots);
    A = A + A';

    % Degree vector
    D = A * ones(nRobots, 1);

    % Maximum Degree Weighting
    FStore = F;
    aStore = a;
    for i = 1:nRobots
        for j = 1:nRobots
            if A(i, j) == 1
                F{i} = F{i} + 1/(1+max(D))*(FStore{j} - FStore{i});
                a{i} = a{i} + 1/(1+max(D))*(aStore{j} - aStore{i});
            end
        end
    end

end

% Final estimation
for i = 1:nRobots
    robots(i).best_tag_estimation = inv(F{i})*a{i};
    robots(i).tag_estimation_history(k) = robots(i).best_tag_estimation;
end


