function plot_point_cloud(tag, readers, radii, points_clouds, plot_circumferences, window, plot_tag)
    if nargin < 6
        plot_tag = true;
    end
    if nargin < 5
        window = [-100, 100];
    end
    if nargin < 4
        plot_circumferences = true;
    end
    
    figure;
    hold on;
    grid on;
    xlim(window);
    ylim(window);
    
    if plot_circumferences
        for i = 1:length(readers)
            circles = [];
            for r = radii{i}
                circles = [circles, viscircles(readers(i,:), r,'Color','k','LineWidth',1)];
            end
        end
    end
    
    if plot_tag
        plot(tag(1), tag(2), 's', 'Color', 'blue', 'MarkerSize', 10,'LineWidth',2);
    end
    
    for i = 1:length(points_clouds)
        points_cloud = points_clouds{i};
        scatter(points_cloud(:,1), points_cloud(:,2), 10,'o', 'r');
    end
    
    scatter(readers(:,1), readers(:,2),10,'kd', 'filled');
    
    axis equal;
    hold off;
end
