class Node():
    def __init__(self, state, parent, move, depth, stepCost, pathCost, heuristicCost):
        self.state = state
        self.parent = parent    # parent node
        self.move = move        # possible moves
        self.depth = depth      # depth of the node in the tree
        self.stepCost = stepCost    # g(n), cost to take each step
        self.pathCost = pathCost    # current cost, cost to reach the current node
        self.heuristicCost = heuristicCost # h(n), cost to reach goal state from current node

        # children node
        self.moveUP = None
        self.moveDOWN = None
        self.moveLEFT = None
        self.moveRIGHT = None

    def findEmptySpace(self, state):
        index = []
        print(self.state)
        blankSpace = state.index(0)
        print(blankSpace)
        if (blankSpace < 3):
            index.append(0)
            index.append(blankSpace)
        elif (blankSpace < 6):
            index.append(1)
            index.append(blankSpace - 3)
        else:
            index.append(2)
            index.append(blankSpace - 6)
        return index
        
    # Function to determine possible move 
    def possibleMoves(self, state):
        possMove = {'UP', 'DOWN','LEFT','RIGHT'}    # SET of possible move
        blankSpace = state.index(0)                 # Location of the empty space
        if (blankSpace % 3 == 0):                   # Check if it is in the first (left) colummn
            possMove.remove('RIGHT')                 # Can't go left if in left column
        if (blankSpace % 3 == 2):                   # Check if it is in the 3rd (right) column
            possMove.remove("LEFT")                # Can't go right if in the right column
        if (blankSpace / 3 < 1):                    # Check if it is in the 1st (top) row
            possMove.remove("DOWN")                 # Tiles above can't go down if the bspace in the top row
        if (blankSpace / 3 >= 2):                   # Check if it is in the 3rd (bottom) row
            possMove.remove("UP")                   # Tiles can't go up if bspace in the bottom row
        return possMove

    # Function swapping the empty space with a space
    def swap(self, state, move):
        state = list(state)
        blankSpace = state.index(0)                     # Locate the empty space
        if (move == 'UP'):                              # Swap space of the empty space and the tile below
            state[blankSpace], state[blankSpace + 3] = state[blankSpace + 3], state[blankSpace]  
        if (move == 'DOWN'):                            # Swap space of the empty space and the tile above
            state[blankSpace], state[blankSpace - 3] = state[blankSpace - 3], state[blankSpace]  
        if (move == 'LEFT'):                            # Swap space of the empty space and the tile to the right
            state[blankSpace], state[blankSpace + 1] = state[blankSpace + 1], state[blankSpace]  
        if (move == 'RIGHT'):                           # Swap space of the empty space and the tile to the left
            state[blankSpace], state[blankSpace - 1] = state[blankSpace - 1], state[blankSpace]  
        return tuple(state)

    def moveUp(self):
        possibleMoves = self.possibleMoves(self.state)
        if("UP" not in possibleMoves):
            return False
        else:
            lowerValue = self.state[self.state.index(0) + 3]
            newState = self.swap(self.state,'UP')
            return newState, lowerValue

    def moveDown(self):
        possibleMoves = self.possibleMoves(self.state)
        if("DOWN" not in possibleMoves):
            return False
        else:
            upperValue = self.state[self.state.index(0) - 3]
            newState = self.swap(self.state,'DOWN')
            return newState, upperValue

    def moveLeft(self):
        possibleMoves = self.possibleMoves(self.state)
        if("LEFT" not in possibleMoves):
            return False
        else:
            rightValue = self.state[self.state.index(0) + 1]
            newState = self.swap(self.state,'LEFT')
            return newState, rightValue

    def moveRight(self):
        possibleMoves = self.possibleMoves(self.state)
        if("RIGHT" not in possibleMoves):
            return False
        else:
            leftValue = self.state[self.state.index(0) - 1]
            newState = self.swap(self.state,'RIGHT')
            return newState, leftValue

    def printPath(self):
        # FILO stacks the trace
        stateTrace = [self.state]
        moveTrace = [self.move]
        depthTrace = [self.depth]
        stepCostTrace = [self.stepCost]
        pathCostTrace = [self.pathCost]
        heuristicCostTrace = [self.heuristicCost]

        # add node info as tracing backup the tree
        while (self.parent):
            self = self.parent
            stateTrace.append(self.state)
            moveTrace.append(self.move)
            depthTrace.append(self.depth)
            stepCostTrace.append(self.stepCost)
            pathCostTrace.append(self.pathCost)
            heuristicCostTrace.append(self.heuristicCost)

        # print the path
        counter = 0
        while (stateTrace):
            print("step:", counter)
            print(stateTrace.pop())
            print('Move: ', moveTrace.pop())
            print('Depth: ', str(depthTrace.pop()))
            print('Step cost: ', str(stepCostTrace.pop()))
            print('Total cost: ', str(pathCostTrace.pop()+heuristicCostTrace.pop()))
            counter+=1 

    def bfs(self, goalState):
        queue =[self]       # queue of unvisited node
        nodePopped = 0      # number of popped node
        queueMaxLength = 1  # max numbder of node in the queue
        depthQueue = [0]    # queue of depth
        pathCostQueue = [0] # queue for path cost
        visited = set([])   # visited node

        while queue:
            if (len(queue) > queueMaxLength):   # Update the max queue length
                queueMaxLength = len(queue)

            currentNode = queue.pop(0)  # Pop the first node
            #print (currentNode.state)
            nodePopped += 1
            currentDepth = depthQueue.pop(0) # Pop the depth for current node
            currentPathCost = pathCostQueue.pop(0)  #Pop the path cost
            visited.add(tuple(currentNode.state))
            if(currentNode.state == tuple(goalState)):
                currentNode.printPath()
                print("Number of Node popped: ", nodePopped)
                print("Max Length: ", str(queueMaxLength))
                return True
            else:
                #print(currentNode.possibleMoves(currentNode.state))
                if ('UP' in currentNode.possibleMoves(currentNode.state)):        # See if lower tile move up is good
                    newState, lowerValue = currentNode.moveUp()
                    
                    if (newState not in visited):   # Check if the resulting node is already visited
                        # Create child node
                        currentNode.moveUP = Node(state = newState, parent = currentNode, move="UP", depth = currentDepth + 1,
                                                    stepCost=lowerValue, pathCost=currentPathCost+lowerValue, heuristicCost=0)
                        queue.append(currentNode.moveUP)
                        depthQueue.append(currentDepth+1)
                        pathCostQueue.append(currentPathCost + lowerValue)

                if ('DOWN' in currentNode.possibleMoves(currentNode.state)):      # See if the upper tile move down is good
                    newState, upperValue = currentNode.moveDown()

                    if (newState not in visited):   # Check if the resulting node is already visited
                        # Create child node
                        currentNode.moveDOWN = Node(state = newState, parent = currentNode, move="DOWN", depth = currentDepth + 1,
                                                    stepCost=upperValue, pathCost=currentPathCost+upperValue, heuristicCost=0)
                        queue.append(currentNode.moveDOWN)
                        depthQueue.append(currentDepth+1)
                        pathCostQueue.append(currentPathCost + upperValue)   

                if ('LEFT' in currentNode.possibleMoves(currentNode.state)):      # See if the right tile move left is good
                    newState, rightValue = currentNode.moveLeft()

                    if (newState not in visited):   # Check if the resulting node is already visited
                        # Create child node
                        currentNode.moveLEFT = Node(state = newState, parent = currentNode, move="LEFT", depth = currentDepth + 1,
                                                    stepCost=rightValue, pathCost=currentPathCost+rightValue, heuristicCost=0)
                        queue.append(currentNode.moveLEFT)
                        depthQueue.append(currentDepth+1)
                        pathCostQueue.append(currentPathCost + rightValue)    

                if ('RIGHT' in currentNode.possibleMoves(currentNode.state)):      # See if the left tile move right is good
                    newState, leftValue = currentNode.moveRight()

                    if (newState not in visited):   # Check if the resulting node is already visited
                        # Create child node
                        currentNode.moveRIGHT = Node(state = newState, parent = currentNode, move="RIGHT", depth = currentDepth + 1,
                                                    stepCost=leftValue, pathCost=currentPathCost+leftValue, heuristicCost=0)
                        queue.append(currentNode.moveRIGHT)
                        depthQueue.append(currentDepth+1)
                        pathCostQueue.append(currentPathCost + leftValue)

    def dfs(self, goalState): 
        queue =[self]       # queue of unvisited node
        nodePopped = 0      # number of popped node
        queueMaxLength = 1  # max numbder of node in the queue
        depthQueue = [0]    # queue of depth
        pathCostQueue = [0] # queue for path cost
        visited = set([])   # visited node

        while queue:
            if (len(queue) > queueMaxLength):   # Update the max queue length
                queueMaxLength = len(queue)

            currentNode = queue.pop(0)  # Pop the first node
            #print (currentNode.state)
            nodePopped += 1
            currentDepth = depthQueue.pop(0) # Pop the depth for current node
            currentPathCost = pathCostQueue.pop(0)  #Pop the path cost
            visited.add(tuple(currentNode.state))
            if(currentNode.state == tuple(goalState)):
                currentNode.printPath()
                print("Number of Node popped: ", nodePopped)
                print("Max Length: ", str(queueMaxLength))
                return True
                
            else:
                #print(currentNode.possibleMoves(currentNode.state))
                if ('UP' in currentNode.possibleMoves(currentNode.state)):        # See if lower tile move up is good
                    newState, lowerValue = currentNode.moveUp()
                    
                    if (newState not in visited):   # Check if the resulting node is already visited
                        # Create child node
                        currentNode.moveUP = Node(state = newState, parent = currentNode, move="UP", depth = currentDepth + 1,
                                                    stepCost=lowerValue, pathCost=currentPathCost+lowerValue, heuristicCost=0)
                        queue.insert(0, currentNode.moveUP)
                        depthQueue.insert(0, currentDepth+1)
                        pathCostQueue.insert(0, currentPathCost + lowerValue)

                if ('DOWN' in currentNode.possibleMoves(currentNode.state)):      # See if the upper tile move down is good
                    newState, upperValue = currentNode.moveDown()

                    if (newState not in visited):   # Check if the resulting node is already visited
                        # Create child node
                        currentNode.moveDOWN = Node(state = newState, parent = currentNode, move="DOWN", depth = currentDepth + 1,
                                                    stepCost=upperValue, pathCost=currentPathCost+upperValue, heuristicCost=0)
                        queue.insert(0, currentNode.moveDOWN)
                        depthQueue.insert(0, currentDepth+1)
                        pathCostQueue.insert(0, currentPathCost + upperValue)   

                if ('LEFT' in currentNode.possibleMoves(currentNode.state)):      # See if the right tile move left is good
                    newState, rightValue = currentNode.moveLeft()

                    if (newState not in visited):   # Check if the resulting node is already visited
                        # Create child node
                        currentNode.moveLEFT = Node(state = newState, parent = currentNode, move="LEFT", depth = currentDepth + 1,
                                                    stepCost=rightValue, pathCost=currentPathCost+rightValue, heuristicCost=0)
                        queue.insert(0, currentNode.moveLEFT)
                        depthQueue.insert(0, currentDepth+1)
                        pathCostQueue.insert(0, currentPathCost + rightValue)    

                if ('RIGHT' in currentNode.possibleMoves(currentNode.state)):      # See if the left tile move right is good
                    newState, leftValue = currentNode.moveRight()

                    if (newState not in visited):   # Check if the resulting node is already visited
                        # Create child node
                        currentNode.moveRIGHT = Node(state = newState, parent = currentNode, move="RIGHT", depth = currentDepth + 1,
                                                    stepCost=leftValue, pathCost=currentPathCost+leftValue, heuristicCost=0)
                        queue.insert(0, currentNode.moveRIGHT)
                        depthQueue.insert(0, currentDepth+1)
                        pathCostQueue.insert(0, currentPathCost + leftValue)

                  

def main():
    #initialState=[1, 4, 3, 7, 0, 6, 5, 8, 2], goalState=[1, 2, 3, 4, 5, 6, 7, 8, 0])
    #initialState=[3,1,2,6,0,8,7,5,4], goalState=[0,1, 2, 3, 4, 5, 6, 7, 8])

    initialState=[1, 4, 3, 7, 0, 6, 5, 8, 2]
    goalState=[1, 2, 3, 4, 5, 6, 7, 8, 0]
    
    rootNode = Node(state=initialState, parent=None, move=None, depth=0, stepCost=0, pathCost=0, heuristicCost=0)
    #rootNode.bfs(goalState)
    rootNode.dfs(goalState)

if (__name__ == '__main__'):
    main()