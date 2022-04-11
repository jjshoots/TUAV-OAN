function NEAR_OBSTACLES = near_obstacles(NEIGHBOUR, OBSERVED_SET, PROX)    
    clearance = PROX;
    
    [x, y] = meshgrid(-clearance:clearance, -clearance:clearance);
    steps = [x(:) y(:)];
    
    NEAR_OBSTACLES = 0;
    
    for i = 1:size(steps, 1)
        if steps(i, 1) ~= 0 && steps(i, 2) ~= 0
            temp_n_neighbour = NEIGHBOUR + steps(i, :);

            if sift(temp_n_neighbour, OBSERVED_SET)
                NEAR_OBSTACLES = NEAR_OBSTACLES + PROX / norm(steps(i, :));
            end
        end
    end
 

%     steps = [1 0; -1 0; 0 1; 0 -1; 1 1; 1 -1; -1 -1; -1 1];
%     
%     NEAR_OBSTACLES = 0;
%     
%     for i = 1:3
%         for j = 1:8
%             % look at neighbour's neighbour
%             temp_n_neighbour = NEIGHBOUR + i .* steps(j, :);
% 
%             % if close to obstacle, add one
%             if sift(temp_n_neighbour, OBSERVED_SET)
%                 NEAR_OBSTACLES = NEAR_OBSTACLES + 1;
%             end
% 
%         end
%     end
end