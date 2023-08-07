function [p1x,p1y,p2x,p2y] = get_intersection(x1,y1,R1,x2,y2,R2)
    d = sqrt((x2-x1)^2 + (y2-y1)^2);

    if d > R1+R2
        ...
    elseif d < abs(R2-R1)
        ...
    elseif d == 0 && R1 == r2
        ...
    elseif d <= R1+R2
        a = (R1^2 - R2^2 + d^2)/(2*d);
        x3 = x1 + a * (x2-x1)/d;
        y3 = y1 + a * (y2-y1)/d;

        h = sqrt(R1^2 - a^2);

        p1x = x3 + h * (y2-y1)/d;
        p1y = y3 - h * (x2-x1)/d;

        p2x = x3 - h * (y2-y1)/d;
        p2y = y3 + h * (x2-x1)/d;

    end
end
