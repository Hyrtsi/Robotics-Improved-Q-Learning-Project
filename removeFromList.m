function points = removeFromList(list, element)

    points = list;

	for i = 1:size(element, 1)
        
        indexList = [];
        
        for j = 1:size(points, 1)
            if element(i,1) == points(j,1) && element(i,2) == points(j,2)
                indexList = [indexList; j];
            end
        end
        
        for index = 1:size(indexList, 1)
            j = indexList(index);
            points(j,:) = [];
        end
                   
	end

end