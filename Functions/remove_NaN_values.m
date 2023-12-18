% function that remove NaN values from a cell array
function new_vector = remove_NaN_values(x)
    new_vector = [];

    for i = 1:length(x)
        if ~isnan(x{i,1})
            new_vector = [new_vector; x{i}(1), x{i}(2)];
        end
    end
end