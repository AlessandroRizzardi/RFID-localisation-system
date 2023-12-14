function target_point = generateRandomPointOnCircumferences(center_of_circle,radius_circle)

    % Generate random angle in radians
    angle = 2 * pi * rand();

    x = center_of_circle(1) + radius_circle * cos(angle);
    y = center_of_circle(2) + radius_circle * sin(angle);

    target_point = [x,y];