function phase_meas = phase_measured(distance,lambda)
    phase = (distance * 4 * pi)/lambda;
    phase_meas = mod(phase, 2*pi);
end

