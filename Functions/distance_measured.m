% function that returns actual distance measured by RFID system
    function distance_meas = distance_measured(distance,lambda)

        phase_meas = phase_measured(distance,lambda) + normrnd(0,0.1);

        distance_meas = (phase_meas * lambda)/(4 * pi) ;
    end