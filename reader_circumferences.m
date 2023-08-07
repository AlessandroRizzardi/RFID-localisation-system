function radii = reader_circumferences(phase_meas,Rmax,lambda)
    dist = 0;
    radii = [];
    n = 0;
    while(dist < Rmax)
        r = lambda*n/2 + (lambda*phase_meas)/(4*pi);
        append(radii,r);
        dist = dist + r;
        n = n + 1;
    end
end

