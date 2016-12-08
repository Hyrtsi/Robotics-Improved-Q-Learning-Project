function direction = getDirection(currentState, nextState)
    % 0 = east
    % 1 = north
    % 2 = west
    % 3 = south
    
    
    dy = currentState(1) - nextState(1);
    dx = currentState(2) - nextState(2);

    if dy == 1
        direction = 1;
    elseif dy == -1
        direction = 3;
    elseif dx == -1
        direction = 0;
    elseif dx == 1
        direction = 2;
    else
        direction = -1;             % Unexpected input detected!
    end
end