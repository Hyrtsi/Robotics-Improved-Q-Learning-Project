function boolValue = allNeighborsLocked(state, lockMatrix, ySize, xSize)
    
    neighbors = [];

    if state(1) + 1 <= ySize
       neighbors = [neighbors; state(1) + 1 state(2)];
    end
       
    if state(2) + 1 <= xSize
       neighbors = [neighbors; state(1) state(2) + 1];
    end
       
    if state(1) > 1
        neighbors = [neighbors; state(1) - 1 state(2)];
    end
       
    if state(2) > 1
       neighbors = [neighbors; state(1) state(2) - 1];
    end
       
    % At this point, we have got the neighbors of the state
    % Obstacles have not been omitted by this function
    % In the main function another function may do it if needed...
    
    boolValue = 1;
    
    for neighbor = neighbors'
        if (lockMatrix(neighbor(1), neighbor(2)) == 0)
            boolValue = 0;
        end
    end

end