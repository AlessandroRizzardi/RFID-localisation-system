%Plot the error along x
error = [];
leg = {};
figure
hold on
for i=1:nRobots
    for k = 1:steps
        if isnan(robots(i).tag_estimation_history{k})
            error(k) = NaN;
        else
            error(k) = abs(robots(i).tag_estimation_history{k}(1) - tag_position(1));
        end
    end
   plot(t, error, 'LineWidth', 2)
   leg{end+1} = ['Robot ', num2str(i)];
end
title('Error in x')
xlabel('Time [s]')
ylabel('Error [m]')
legend(leg)