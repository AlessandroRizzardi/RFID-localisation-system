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
figure(1)
hold on
plot(x_odometry,y_odometry,'b')
plot(x_dynamics,y_dynamics,'r')
plot(tag_position(1),tag_position(2),'r*','MarkerSize',10)
plot(best_tag_estimation_x, best_tag_estimation_y,'b*','MarkerSize',10)
plot(x_coord,y_coord);
legend('Odometry','Dynamics','Tag Position','Best estimation')
xlabel('x [m]')
ylabel('y [m]')
xlim(x_range)
ylim(y_range)