function values = getValues(list, indexList)
    values = [];
    
    for i = 1:size(indexList,1)
        values = [values; list(indexList(i,1),indexList(i,2))];
    end
end