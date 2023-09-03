function phase_noisy = get_measurement(phase_meas)
    phase_noisy = phase_meas + 0.1 * normrnd(0,1);
end

