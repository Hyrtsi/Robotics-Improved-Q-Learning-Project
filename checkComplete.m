function [complete] = checkComplete(L,O)
L_temp = L;

for i = 1:length(O)
    L_temp(O(i,1),O(i,2))= 1;
end

if isequal(L_temp,ones(size(L,1),size(L,2)))
    complete = 1;
else
    complete = 0;
end