%IQL Algorithm

clear all;

% Grid world configuration:
% 
% We should let the matrices be n*m
%
% Note: we follow Matlab syntax and address the matrices as follows:
%
% M(y,x) <<--- " y first then x "

% Semi-poor design: we have put the boundary check into takeAction function
% to avoid doing it here. Price-tag = giving xSize, ySize as a parameter


% ------------------------------------------------------

xSize = 8;
ySize = 8;

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


% The following are (not very interesting) testcases.

%obstacles = [2 2; 3 2; 3 4];
%obstacles = [1 1; 5 5; 3 4;2 3; 5 1];
%obstacles = [1 1; 1 2; 1 3; 1 4; 1 5; 2 1; 3 1; 4 1; 2 2];


obstacles = [3 3];
%obstacles = [3 3; 2 2; 4 3];
%obstacles = [2 2; 3 1; 4 1; 5 1; 5 2];






discount = 0.5;         % Initialize me!


% Initialize iterateList = " All states s for which all neighbors are not locked "

iterateList = [yGoal xGoal];


% ---------------------------------------- %


complete = 0;

while (complete == 0)

    % We might have a huge problem in the current implementation because
    % the size of iterateList is changing over the runs...
    
    % Changed to while-loop which should work.
    % This is a huge design issue and somehow it works for size (5,5) but
    % not other sizes. Poor!
    
    
    %for idx = 1:size(iterateList,1)            % OLD
    while size(iterateList,1) > 0
        
        % For debugging:
        
        %size(iterateList,1)
        %idx
        %iterateList(idx,:)
        
        
        
        %state = iterateList(idx,:);            % OLD
        state = iterateList(1,:);               % Always pick the first :)
        
        
        
        % This loop runs through all elements of iterateList
        % Weird syntax - double check that it works!

        % The next list contains (y,x) coordinates of neighbors, 1-4x
        neighborList = getNeighbors(state, ySize, xSize);

        % Update all the neighbors of current state
        for i = 1:size(neighborList, 1)
            neighbor = neighborList(i,:);
            xPresent = state(2);
            yPresent = state(1);
            xNext = neighbor(2);
            yNext = neighbor(1);

            currentPoint = [yPresent xPresent];
            nextPoint = [yNext xNext];

            % Calculate the distances
            distanceNew = cityblockdistance(nextPoint, goalPoint);
            distancePresent = cityblockdistance(currentPoint, goalPoint);


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

            % Add necessary neighbors to be processed next
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
        end
    end

    % Checking exit condition
    complete = checkComplete(L,obstacles);
        
end
sprintf('Q-Table Complete!')




% % % % 
% Begin path planning phase
% % % %



% With a given Q-table, the following algorihtm makes a path of it.
% We introduce another quality: direction
% Direction = [0,1,2,3]


initState = [yInit xInit];
currentState = initState;
goalState = [yGoal xGoal];

direction = -1;         % INIT

path = [yInit xInit];


maxIterations = 1e4;
iterationIndex = 0;

while ismember(currentState, goalState, 'rows') == 0
    
    iterationIndex = iterationIndex + 1;
    
    if iterationIndex > maxIterations
        sprintf('Path planning stuck!')
        break
    end
    
    clear max;
    
    neighborList = getNeighbors(currentState, ySize, xSize);
    
    % The following function removes obstacles from neighbors
    neighborList = removeFromList(neighborList, obstacles);
    
   
    % Q-values of the neighbors
    neighborValues = getValues(Q, neighborList);
    
    % Determine if we have multiple equally best choices
    indices = find(neighborValues >= max(neighborValues));
    
    % Now we know 1 to 4 indices for equal best choices
    % Minimize the amount of turns...
    
    if direction == -1
        % This is the first run only
        nextState = neighborList(indices(1),:);
        newDirection = getDirection(currentState, nextState);
    else
        % After the first run
        bestIndex = -1;
        bestValue = 1000;
        
        for i = 1:size(indices,1)
            
            % The direction from current state to 
            tempDirection = getDirection(currentState, neighborList(indices(i),:));
            
            % How many times we have to rotate 90deg to get to the state
            nTurns = mod(tempDirection - direction, 3);
           
            if nTurns < bestValue
                bestValue = nTurns;
                bestIndex = indices(i);
            end

        end
          
        newDirection = tempDirection;
        nextState = neighborList(bestIndex,:);
    end

    direction = newDirection;
    
    currentState = nextState;
    path = [path; currentState];
    
end

if iterationIndex < maxIterations
    sprintf('Path Planning Complete!')
end



% We will plot this very poorly...


hold on;
plot(xInit, yInit, 'b*')
plot(xGoal, yGoal, 'm*')
plot(path(:,2), path(:,1), 'k-')
for i = 1:size(obstacles,1)
   plot(obstacles(i,2), obstacles(i,1), 'r*')
end
legend('Initial location', 'Goal', 'Chosen path', 'Obstacles')
grid on;
xlim([-1 xSize + 1])
ylim([-1 ySize + 1])

title('IQL Algorithm Test Run')
xlabel('x')
ylabel('y')

