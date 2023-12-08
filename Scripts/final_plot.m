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
            plot(robots(i).tag_estimation_history{k}(1), robots(i).tag_estimation_history{k}(2), 'b*', 'MarkerSize', 10)
        end    
    
        xlabel('x [m]')
        ylabel('y [m]')

        xlim([-6, 6])
        ylim([-6, 6])

        title(['Step ', num2str(k)])
        %pause(.001)
        drawnow
    end
end

if DRAW == true
    
    figure(1)
    hold on
    for ROBOT=1:nRobots
        plot(robots(ROBOT).best_tag_estimation(1), robots(ROBOT).best_tag_estimation(2), 'b*','MarkerSize',10)
    end
    plot(tag_position(1),tag_position(2),'r*','MarkerSize',10)
    plot(x_coord,y_coord);
    xlabel('x [m]')
    ylabel('y [m]')

end

% Plot the error
%figure
%hold on
%for i=1:nRobots
%    plot(t, robots(i).tag_estimation_history{1,:} - tag_position(1), 'LineWidth', 2)
%    leg{end+1} = ['Robot ', num2str(i)];
%end
%title('Error in x')
%xlabel('Time [s]')
%ylabel('Error [m]')
%legend(leg)
%
%figure
%hold on
%for i=1:nRobots
%    plot(t, robots(i).tag_estimation_history{2,:} - tag_position(2), 'LineWidth', 2)
%    leg{end+1} = ['Robot ', num2str(i)];
%end
%title('Error in y')
%xlabel('Time [s]')
%ylabel('Error [m]')
%legend(leg)