function points = circumferences_interceptions(radii_a,radii_b,reader_a,reader_b)
    points = [];

    for i = 1:len(radii_a)
        for j = 1:len(radii_b)
            intersections = get_intersection(reader_a(1),reader_a(2),radii_a(i),reader_b(1),reader_b(2),radii_b(j));
            if ...
                p1x,p1y,p2x,p2y = intersections;
                P1 = [round(p1x,3), round(p1y,3)];
                P2 = [round(p2x,3), round(p2y,3)];

                if ~ismember(P1,points)
                    append(points,P1)
                end

                if ~ismember(P2,points)
                    append(points,P2)
                end

            end
        end
    end

end





    
end

