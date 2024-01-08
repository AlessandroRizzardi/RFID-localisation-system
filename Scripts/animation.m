angle = 0:0.1:2*pi;
rad = max_range; %[m]
x_coord = tag_position(1) + rad*cos(angle);
y_coord = tag_position(2) + rad*sin(angle);


% vector of different colors and markers for the plots
markerStyles = {'diamond', 'pentagram', 'o', 'square', 'v'};

% I need to plot the tag position and the robot position with different colors and markers
color_plot= lines(nRobots); % Vector of different colors

% Animations
for k = 1:steps
    leg = {};
    figure(1)
    clf
    hold on

    plot(tag_position(1),tag_position(2),'Marker', markerStyles{4},'MarkerSize',10)
    plot(x_coord,y_coord, '--g');
    leg{1} = 'Tag';
    leg{2} = 'Tag Range';

    for ROBOT=1:nRobots
        plot(robots(ROBOT).odometry_history{k,1}(1), robots(ROBOT).odometry_history{k,1}(2), 'Color', color_plot(ROBOT, :), 'Marker', markerStyles{1})
        % plot(robots(ROBOT).dynamics_history{k,1}(1), robots(ROBOT).dynamics_history{k,1}(2),'Color', color_plot(ROBOT, :), 'Marker', markerStyles{3})
        plot(robots(ROBOT).tag_estimation_history{k,1}(1), robots(ROBOT).tag_estimation_history{k,1}(2), 'Color', color_plot(ROBOT, :), 'Marker', markerStyles{2}, 'MarkerSize', 7)
        %legend
        leg{end+1} = ['Robot', num2str(ROBOT)];
        leg{end+1} = ['TagEstimation ', num2str(ROBOT)];
    end    

    xlabel('x [m]')
    ylabel('y [m]')

    xlim([-6, 6])
    ylim([-6, 6])

    title(['Step ', num2str(k)])
    legend(leg)
    %legend(leg)
    pause(.01)
    drawnow
end