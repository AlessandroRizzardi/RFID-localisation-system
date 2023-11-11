angle = 0:0.1:2*pi;
rad = max_range; %[m]
x_coord = tag_position(1) + rad*cos(angle);
y_coord = tag_position(2) + rad*sin(angle);



color_drawing = ['r--'; 'b--' ;'g--'];
color_animation = ['ro'; 'bo' ;'go'];
color_tag = ['r*'; 'b*' ;'g*'];

% Animations
if ANIMATION == true
    for k = 1:steps
        figure(1)
        clf
        hold on

        plot(tag_position(1),tag_position(2),'r*','MarkerSize',10)
        plot(x_coord,y_coord, '--g');
        
        for i=1:nRobots
            plot(robots(i).odometry_history{k,1}(1), robots(i).odometry_history{k,1}(2), 'ro')
            plot(robots(i).dynamics_history{k,1}(1), robots(i).dynamics_history{k,1}(2),'bo','MarkerSize', 10, 'LineWidth',2)
            plot(robots(i).tag_estimation_history{k}(1), robots(i).tag_estimation_history{k}(2), 'c*', 'MarkerSize', 10)
        end    
    
        xlabel('x [m]')
        ylabel('y [m]')

        xlim([-5, 5])
        ylim([-5, 5])

        title(['Step ', num2str(k)])
        pause(.001)
    end
end

if DRAW == true
    
    figure(1)
    hold on
    for ROBOT=1:nRobots
        x_odometry = zeros(length(robots(ROBOT).odometry_history),1);
        y_odometry = zeros(length(robots(ROBOT).odometry_history),1);
        x_dynamics = zeros(length(robots(ROBOT).dynamics_history),1);
        y_dynamics = zeros(length(robots(ROBOT).dynamics_history),1);

        for i = 1:length(robots(ROBOT).odometry_history)
            x_odometry(i) = robots(ROBOT).odometry_history{i,1}(1);
            y_odometry(i) = robots(ROBOT).odometry_history{i,1}(2);

            x_dynamics(i) = robots(ROBOT).dynamics_history{i,1}(1);
            y_dynamics(i) = robots(ROBOT).dynamics_history{i,1}(2);
        end

        angle = 0:0.1:2*pi;
        rad = max_range;%[m]
        x_coord = tag_position(1) + rad*cos(angle);
        y_coord = tag_position(2) + rad*sin(angle);
        
        x_coord_map = radius_map*cos(angle);
        y_coord_map = radius_map*sin(angle);


        % Plottings
        
        
        %plot(x_odometry,y_odometry,'b')
        plot(x_dynamics,y_dynamics, color_drawing(ROBOT))
        plot(tag_position(1),tag_position(2),'r*','MarkerSize',10)
        plot(x_coord,y_coord);
        plot(x_coord_map, y_coord_map);
        xlabel('x [m]')
        ylabel('y [m]')
    end

end
