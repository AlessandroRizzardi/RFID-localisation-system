function points = circumferences_interceptions(radiia, radiib, readera, readerb)
    points = [];
    
    for i = 1:length(radiia)
        r1 = radiia(i);
        for j = 1:length(radiib)
            r2 = radiib(j);
            [p1x, p1y, p2x, p2y] = get_intersections(readera(1), readera(2), r1, readerb(1), readerb(2), r2);
            intersections = [p1x, p1y, p2x, p2y];
            if ~isnan(intersections(1))
                p1x = round(intersections(1), 3);
                p1y = round(intersections(2), 3);
                p2x = round(intersections(3), 3);
                p2y = round(intersections(4), 3);
                
                P1 = [p1x, p1y];
                P2 = [p2x, p2y];
                
                if ~ismember(P1, points)
                    points = [points; P1];
                end
                if ~ismember(P2, points)
                    points = [points; P2];
                end
            end
        end
    end
end

