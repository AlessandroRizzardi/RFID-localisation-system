x_odometry = zeros(length(odometry_history),1);
y_odometry = zeros(length(odometry_history),1);
x_dynamics = zeros(length(dynamics_history),1);
y_dynamics = zeros(length(dynamics_history),1);

for i = 1:length(odometry_history)
    x_odometry(i) = odometry_history{i,1}(1);
    y_odometry(i) = odometry_history{i,1}(2);

    x_dynamics(i) = dynamics_history{i,1}(1);
    y_dynamics(i) = dynamics_history{i,1}(2);
end

angle = 0:0.1:2*pi;
rad = max_range; %[m]
x_coord = tag_position(1) + rad*cos(angle);
y_coord = tag_position(2) + rad*sin(angle);

% Plottings
for k = 1:steps
    figure(1)
    clf
    hold on
    plot(x_odometry(k),y_odometry(k),'bo')
    plot(x_dynamics(k),y_dynamics(k),'ro','MarkerSize', 10, 'LineWidth',2)
    plot(tag_position(1),tag_position(2),'r*','MarkerSize',10)
    plot(tag_estimation_history(k,1),tag_estimation_history(k,1),'b*','MarkerSize',10)
    plot(x_coord,y_coord, '--g');
    %legend('Odometry','Dynamics','Tag Position','Best estimation')
    xlabel('x [m]')
    ylabel('y [m]')
    xlim([-5,5])
    ylim([-5,5])
    title(['Step ', num2str(k)])
    pause(.01)
    
end