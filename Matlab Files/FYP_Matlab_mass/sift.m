function SIFT = sift(target, list)
    SIFT = 0;
    for i = 1:size(list, 1)
        if ~norm(target - list(i, :))
            SIFT = 1;
            break;
        end

        if i == size(list, 1)
            SIFT = 0;
        end
    end
end