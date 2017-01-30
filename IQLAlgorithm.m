% IQL Algorithm

clear all;
clf;

% Note: we follow Matlab syntax and address the matrices as follows:
%
% M(y,x) <<--- " y first then x "

% ------------------------------------------------------

% Initial values

xSize = 8;
ySize = 8;

xGoal = 6;
yGoal = 6;

xInit = 1;
yInit = 1;

L = zeros(ySize, xSize);
Q = zeros(ySize, xSize);

L(yGoal, xGoal) = 1;
Q(yGoal, xGoal) = 100;

initialPoint = [yInit xInit];
currentPoint = initialPoint;
goalPoint = [yGoal xGoal];

% In the current implementation, discount actually makes no difference.
discount = 0.5;

% Initialize iterateList = " All states s for which all neighbors are not locked "
iterateList = [yGoal xGoal];



% % Obstacle configurations % %


% To test behavior when confronting a wall - or two!

wall1 = [3 3; 2 3; 4 3; 6 3; 7 3; 8 3; 5 3; 1 6];
wall2 = [1 5; 2 5; 3 5; 4 5; 5 5; 6 5; 7 5];

%obstacles = [wall1; wall2];

% To emerge maze-like turning behavior
%
% Note that with this kind of obstacles, the chosen path is far from
% optimal.
% We see that at (4,4) the algorithm rather selects the route
% with less turning than sees the big picture. Indeed, the algorithm falls
% for local solutions.
% 1) Is this expected for the algorithm?
% 2) How do we improve our solution? ( = either implement the algorithm correctly
% or improve the algorithm )
%obstacles = [2 2; 3 1; 1 4; 3 3; 2 5; 6 4; 5 5; 6 3; 6 2; 5 2];











% ---------------------------------------- %


tic;

complete = 0;

while (complete == 0)
    while size(iterateList,1) > 0
        state = iterateList(1,:);               % Always pick the first

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

toc;


% % % % 
% Path planning
% % % %



% With a given Q-table, the following algorithm generates a path of it.

% Initialization
initState = [yInit xInit];
currentState = initState;
goalState = [yGoal xGoal];
direction = -1;

% We save the chosen path
path = [yInit xInit];

% To prevent too long runs
maxIterations = 1e3;
iterationIndex = 0;

% Make a copy of the Q-table to be able to compare
TempQ = Q;



tic;

while ismember(currentState, goalState, 'rows') == 0
    
    iterationIndex = iterationIndex + 1;
    
    if iterationIndex > maxIterations
        sprintf('Path planning stuck!')
        break
    end
    
    clear max;
    
    % Neighbors of the current state
    neighborListRaw = getNeighbors(currentState, ySize, xSize);
    
    % The following function removes obstacles from neighbors
    neighborList = removeFromList(neighborListRaw, obstacles);
    

    % Q-values of the neighbors
    neighborValues = getValues(TempQ, neighborList);
    
    % Determine if we have multiple equally best choices
    indices = find(neighborValues >= max(neighborValues));
    
    % Now we know 1 to 2 indices for equal best choices
    % Minimize the amount of turns in the next loop 
    
    if direction == -1
        % This is the first run only
        nextState = neighborList(indices(1),:);
        newDirection = getDirection(currentState, nextState);
    else
        % After the first run

        % The following variable is for finding the least amount of turns
        % required. Initialized to a "big" number.
        leastTurns = 1000;
        
        for i = 1:size(indices,1)
            
            % The direction from current state to 
            tempDirection = getDirection(currentState, neighborList(indices(i),:));
            
            % Amount of turns required from current direction to the new direction
            nTurns = calculateTurns(direction, tempDirection);
            
            % A simple method for finding which of the two directions takes
            % less turns.
            if nTurns < leastTurns
                leastTurns = nTurns;
                newDirection = tempDirection;
                nextState = neighborList(indices(i),:);
            end
        end
    end

    % We permit going the same route until it is necessary
    TempQ(nextState(1), nextState(2)) = 0;
    
    direction = newDirection;
    
    currentState = nextState;
    path = [path; currentState];
    
end




if iterationIndex < maxIterations
    sprintf('Path Planning Complete!')
end

toc;



% % % % % % % % % % % % % % % % % % % % % % % % %

% Plotting the path


hold on;
plot(xInit, yInit, 'b*')
plot(xGoal, yGoal, 'm*')
plot(path(:,2), path(:,1), 'k-')
for i = 1:size(obstacles,1)
   plot(obstacles(i,2), obstacles(i,1), 'r*')
end
legend('Initial location', 'Goal', 'Chosen path', 'Obstacles')
grid on;
xlim([0 xSize + 1])
ylim([0 ySize + 1])

title('IQL Algorithm Test Run')
xlabel('x')
ylabel('y')

