function distance = get_tagDistance(tag,reader)
    distance = sqrt((tag(1)-reader(1))^2 + (tag(2)-reader(2))^2);
end

