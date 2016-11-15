function neighbors = getNeighbors(state, ySize, xSize)
    
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

end