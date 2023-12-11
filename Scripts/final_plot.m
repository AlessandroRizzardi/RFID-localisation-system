angle = 0:0.1:2*pi;
rad = max_range; %[m]
x_coord = tag_position(1) + rad*cos(angle);
y_coord = tag_position(2) + rad*sin(angle);


% vector of different colors and markers for the plots
markerStyles = {'d', 'pentagram', 'o', '^', 'v'};

% I need to plot the tag position and the robot position with different colors and markers (possibly more than 10)
color_plot= lines(nRobots); % Vector of different colors


% Animations
if ANIMATION == true
    for k = 1:steps
        leg = {};
        figure(1)
        clf
        hold on

        plot(tag_position(1),tag_position(2),'Marker', markerStyles{1},'MarkerSize',10)
        plot(x_coord,y_coord, '--g');
        leg{1} = 'Tag';
        leg{2} = 'Tag Range';

        for ROBOT=1:nRobots
            %plot(robots(ROBOT).odometry_history{k,1}(1), robots(ROBOT).odometry_history{k,1}(2), 'Color', color_plot(ROBOT, :), 'Marker', markerStyles{3})
            plot(robots(ROBOT).dynamics_history{k,1}(1), robots(ROBOT).dynamics_history{k,1}(2),'Color', color_plot(ROBOT, :), 'Marker', markerStyles{3})
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
        %legend(leg)
        pause(.01)
        drawnow
    end
end

if DRAW == true
    leg = {};
    figure(1)
    hold on
    plot(tag_position(1),tag_position(2),'Marker', markerStyles{1},'MarkerSize',10)
    plot(x_coord,y_coord);
    leg{1} = 'Tag';
    leg{2} = 'Tag Range';
    for ROBOT=1:nRobots
        % matrix_position = cell2mat(robots(ROBOT).odometry_history);
        % plot(matrix_position(:,1), matrix_position(:,2), 'Color', color_plot(ROBOT, :))
        plot(robots(ROBOT).best_tag_estimation(1), robots(ROBOT).best_tag_estimation(2), 'Color', color_plot(ROBOT, :), 'Marker', markerStyles{2}, 'MarkerSize', 7)
        % leg{end+1} = ['Robot', num2str(ROBOT)];
        leg{end+1} = ['TagEstimation ', num2str(ROBOT)];
    end
    xlim([-6, 6])
    ylim([-6, 6])
    xlabel('x [m]')
    ylabel('y [m]')
    title('Final Estimate')
    legend(leg)
end

% Plot the error
% error = [];
% figure
% hold on
% for i=1:nRobots
%     for k = 1:steps
%         if isnan(robots(i).tag_estimation_history{k})
%             error(k) = NaN;
%         else
%             error(k) = robots(i).tag_estimation_history{k}(1) - tag_position(1);
%         end
%     end
%    plot(t, error, 'LineWidth', 2)
%    leg{end+1} = ['Robot ', num2str(i)];
% end
% title('Error in x')
% xlabel('Time [s]')
% ylabel('Error [m]')
% legend(leg)
% 
% error = [];
% figure
% hold on
% for i=1:nRobots
%    for k = 1:steps
%         if isnan(robots(i).tag_estimation_history{k})
%             error(k) = NaN;
%         else
%             error(k) = robots(i).tag_estimation_history{k}(2) - tag_position(2);
% 
%         end
%    end
%    plot(t, error, 'LineWidth', 2)
%    leg{end+1} = ['Robot ', num2str(i)];
% end
% title('Error in y')
% xlabel('Time [s]')
% ylabel('Error [m]')
% legend(leg)