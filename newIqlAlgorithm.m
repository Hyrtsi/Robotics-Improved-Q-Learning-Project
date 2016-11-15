%IQL Algorithm


% Grid world configuration:
% 
% How actions [1,2,3,4] correspond to movement (direction)
% We should let the matrices be n*m
%
% Note: we follow Matlab syntax and address the matrices as follows:
%
% M(y,x) <<--- " y first then x "

% Semi-poor design: we have put the boundary check into takeAction function
% to avoid doing it here. Price-tag = giving xSize, ySize as a parameter


% ------------------------------------------------------

% Try #2:
% Outlines:
% 
% Reference for algorithm in paper - if-loop in the paper may be broken
% We first try only satisfying the properties
% Also new approach: Just somehow visit all states and update their L%Q status




xSize = 5;
ySize = 5;

xGoal = 5;
yGoal = 3;

xInit = 1;
yInit = 1;

L = zeros(ySize, xSize);
Q = zeros(ySize, xSize);

L(yGoal, xGoal) = 1;
Q(yGoal, xGoal) = 100;

initialPoint = [yInit xInit];
currentPoint = initialPoint;
goalPoint = [yGoal xGoal];

obstacles = [2 2; 3 2; 3 4];

discount = 0.5;         % Initialize me!






% Initialize iterateList = " All states s for which A(s) == 0 "

iterateList = [yGoal xGoal];


% ---------------------------------------- %


complete = 0;            % Placeholder...

while (complete == 0)
    

    for idx = 1:size(iterateList,1)
        state = iterateList(idx,:);
        % This loop runs through all elements of iterateList
        % Weird syntax - double check that it works!

        % The next list contains (y,x) coordinates of neighbors, 1-4x
        neighborList = getNeighbors(state, ySize, xSize);   % TBC ;)

        for i = 1:size(neighborList, 1)
            neighbor = neighborList(i,:);
            xPresent = state(2);
            yPresent = state(1);
            xNext = neighbor(2);
            yNext = neighbor(1);

            currentPoint = [yPresent xPresent];
            nextPoint = [yNext xNext];

            % Calculate the distances
            distanceNew = cityblockdistance(nextPoint, goalPoint)
            distancePresent = cityblockdistance(currentPoint, goalPoint)


            % Update L, Q here...

            if distanceNew < distancePresent
                if L(yPresent, xPresent) == 0 && L(yNext, xNext) == 1       %% Property 1
                    Q(yPresent, xPresent) = discount * Q(yNext, xNext);
                    L(yPresent, xPresent) = 1;
                elseif L(yPresent, xPresent) == 1 && L(yNext, xNext) == 0	%% Property 2
                    Q(yNext, xNext) = Q(yPresent, xPresent) / discount;
                    L(yNext, xNext) = 1;
                end
            elseif distancePresent < distanceNew
                if L(yNext, xNext) == 0 && L(yPresent, xPresent) == 1       %% Property 3
                    Q(yNext, xNext) = discount * Q(yPresent, xPresent);
                    L(yNext, xNext) = 1;
                elseif L(yNext, xNext) == 1 && L(yPresent, xPresent) == 0	%% Property 4
                    Q(yPresent, xPresent) = Q(yNext, xNext) / discount;
                    L(yPresent, xPresent) = 1;
                end
            end

            % L, Q updated.

            if allNeighborsLocked(neighbor, L, ySize, xSize) == 0
                iterateList = [iterateList; neighbor];
            end
        end

        % All neighbors iterated.
        % At this point the following should ALWAYS be true

        if allNeighborsLocked(state, L, ySize, xSize) == 1

            % We remove current state from iterateList:

            LIA = ismember(iterateList, state, 'rows');
            myIndex = find(LIA,1);
            iterateList(myIndex,:) = [];
        else
            1+100
            % We have a problem here......
        end
    end

    % Checking exit condition
    complete = checkComplete(L,obstacles);
        
end
sprintf('Complete!')

