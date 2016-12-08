function points = removeFromList(list, element)

    points = [];

    for j = 1:size(list, 1)
	
        matchesObstacle = 0;
        
        for i = 1:size(element, 1)
            if element(i,1) == list(j,1) && element(i,2) == list(j,2)
                matchesObstacle = 1;
            end
        end
        
        if matchesObstacle == 0
            points = [points; list(j,:)];
        end
    end

    
    
    
    
    
    
    
    
    
    
    
    
    
end