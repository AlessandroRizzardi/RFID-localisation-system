clear 
close all
clc

reader1 = [30, 10];
reader2 = [-35, 0];
reader3 = [-20, 25];
reader4 = [0, -20];
readers = [reader1; reader2; reader3];  % Use semicolons to create a matrix
tag = [0, 0];
NOISE = true;

Rmax = 70;
lamda = 20;

distance1 = get_tagDistance(tag, reader1);
distance2 = get_tagDistance(tag, reader2);
distance3 = get_tagDistance(tag, reader3);
distance4 = get_tagDistance(tag, reader4);

phase_measured1 = phase_measured(distance1, lamda);
phase_measured2 = phase_measured(distance2, lamda);
phase_measured3 = phase_measured(distance3, lamda);
phase_measured4 = phase_measured(distance4, lamda);

if NOISE == true
    phase_measured1 = get_measurement(phase_measured1);
    phase_measured2 = get_measurement(phase_measured2);
    phase_measured3 = get_measurement(phase_measured3);
    phase_measured4 = get_measurement(phase_measured4);
end

radii1 = reader_circumferences(phase_measured1, Rmax, lamda);
radii2 = reader_circumferences(phase_measured2, Rmax, lamda);
radii3 = reader_circumferences(phase_measured3, Rmax, lamda);
radii4 = reader_circumferences(phase_measured4, Rmax, lamda);
radii = {radii1, radii2, radii3};  % Use curly braces to create a cell array

points_cloud1 = circumferences_interceptions(radii1, radii2, reader1, reader2);
points_cloud2 = circumferences_interceptions(radii1, radii3, reader1, reader3);
points_cloud3 = circumferences_interceptions(radii2, radii3, reader2, reader3);
points_cloud4 = circumferences_interceptions(radii1, radii4, reader1, reader4);
points_clouds = {points_cloud1, points_cloud2, points_cloud3};  % Use curly braces to create a cell array


plot_circumferences=true;
window=[-70,70];
plot_tag=true;

plot_point_cloud(tag, readers, radii, points_clouds, plot_circumferences, window, plot_tag)

