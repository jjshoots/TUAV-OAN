function idx = locate(target, map)
    if sift(target, map)
        for idx = 1:size(map, 1)
            if target(1) == map(idx, 1)
                if target(2) == map(idx, 2)
                    break;
                end
            end
        end
    else
        % if node is not found in map,
        % return NaN so the whole code breaks. ._.
        idx = NaN;
    end
end