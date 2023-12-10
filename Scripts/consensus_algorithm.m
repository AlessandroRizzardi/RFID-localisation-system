%number of consensus protocol messages exchanged by the nodes in the network
m = 100;

% Initialisation
F = cell(nRobots, 1);
a = cell(nRobots, 1);

for r = 1:nRobots
    if robots(r).init_flag == true 
        j = robots(r).instance_selected;
        zi = robots(r).best_tag_estimation;
        Hi = eye(2);
        R_i = tag_measurment_uncertainty(MHEKFs(r,j).P, robots(r));

        F{r} = Hi'*inv(R_i)*Hi;
        a{r} = Hi'*inv(R_i)*zi;

    else
        zi = 0;
        Hi = 0;
        F{r} = 0;
        a{r} = 0;
    end
end


% Topology matrix: 1 if there is a link between the nodes when the robots are inside the communication range (tag_vector is true), 0 otherwise
for r = 1:nRobots
    for t = 1:nRobots
        if r ~= t
            if tag_flag_vector(r) == true && tag_flag_vector(t) == true
                A(r, t) = 1;
            else
                A(r, t) = 0;
            end
        else
            A(r, t) = 0;
        end
    end
end

% Degree vector
D = A * ones(nRobots, 1);

for q = 1:m
    FStore = F;
    aStore = a;
    % Maximum Degree Weighting
    for r = 1:nRobots
        for t = 1:nRobots
            if A(r, t) == 1
                F{r} = F{r} + 1/(1+max(D))*(FStore{t} - FStore{r});
                a{r} = a{r} + 1/(1+max(D))*(aStore{t} - aStore{r});
            end
        end
    end

end

%Final estimation
for r = 1:nRobots
    if robots(r).init_flag == true 
        robots(r).best_tag_estimation = inv(F{r})*a{r};
        robots(r).tag_estimation_history{k} = inv(F{r})*a{r}; 
    else
        robots(r).best_tag_estimation = [NaN;NaN];
        robots(r).tag_estimation_history{k} = [NaN;NaN];
    end
end


