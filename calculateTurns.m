function nTurns = calculateTurns(direction1, direction2)
    if direction1 == 0
        if direction2 == 3
            nTurns = 1;
        else
            nTurns = direction2;
        end
    elseif direction1 == 1
        nTurns = abs(direction1 - direction2);
    elseif direction1 == 2
        nTurns = abs(direction1 - direction2);
    elseif direction1 == 3
        if direction2 == 0
            nTurns = 1;
        else
            nTurns = direction1 - direction2;
        end
    end

end