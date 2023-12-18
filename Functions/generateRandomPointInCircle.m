function target_point = generateRandomPointInCircle(center_of_circle, radius_circle)
    % Generate random angle in radians
    angle = 2 * pi * rand();

    % Generate random radius within the circle's radius
    radius = radius_circle * sqrt(rand());

    % Calculate the x and y coordinates of the target point
    x = center_of_circle(1) + radius * cos(angle);
    y = center_of_circle(2) + radius * sin(angle);

    % Create the target point as a 2-element vector
    target_point = [x, y];
end
