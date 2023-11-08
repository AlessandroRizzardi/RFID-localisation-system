angle = 0:0.1:2*pi;
rad = max_range; %[m]
x_coord = tag_position(1) + rad*cos(angle);
y_coord = tag_position(2) + rad*sin(angle);

% Animations
for k = 1:steps
    figure(1)
    clf
    hold on
    
    plot(tag_position(1),tag_position(2),'r*','MarkerSize',10)
    plot(x_coord,y_coord, '--g');

    for i=1:nRobots
        plot(robots(i).odometry_history{k,1}(1), robots(i).odometry_history{k,1}(2),'bo')
        plot(robots(i).dynamics_history{k,1}(1), robots(i).dynamics_history{k,1}(2), 'ro','MarkerSize', 10, 'LineWidth',2)

        plot(robots(i).tag_estimation_history(k,1),tag_estimation_history(k,2),'b*', 'MarkerSize', 10)
    end
     
    xlabel('x [m]')
    ylabel('y [m]')

    xlim([-20,20])
    ylim([-20,20])

    title(['Step ', num2str(k)])
    pause(.001)
    
end