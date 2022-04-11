function SIFT = sift(target, list)
    if size(list, 1) < 1
        SIFT = 0;
        return;
    end
    SIFT = ismember(target, list, 'rows');
%     SIFT = 0;
%     for i = 1:size(list, 1)
%         if ~norm(target - list(i, :))
%             SIFT = 1;
%             break;
%         end
% 
%         if i == size(list, 1)
%             SIFT = 0;
%         end
%     end
end