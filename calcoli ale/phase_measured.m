function phase_meas = phase_measured(distance,lambda)
    phase = get_phase(distance,lambda);
    phase_meas = mod(phase, 2*pi);
end

