
from numpy import tile
from Heap import Heap
from utils import *
from mapper import Map
from queue import Queue

class BreadthFirstSearch:

    NO_PATH = -1
    FINISHED = 0
    RUNNING = 1
    def __init__(self, robot, map) -> None:
        self._rb = robot
        self._map = map.map
        self.next_goal_x = 0
        self.next_goal_y = 0

    def getNextGoal(self, force_new_goal = False):
        tile_x, tile_y = getCellCoords(self._rb.position_x, self._rb.position_y)

        # calculate the next tile to pathfind to if the current tile is reached, the next goal is a black tile or a recalculation is forced
        if tile_x == self.next_goal_x and tile_y == self.next_goal_y or self._map.getGroundState(self.next_goal_x,self.next_goal_y) == Map.HOLE or force_new_goal:
            self.next_goal_x, self.next_goal_y = self.getNearestTileFromRobot()

            # if no more goal is found and the robot is standing on the start tile, the run is ended
            if tile_x == self.next_goal_x == 0 and tile_y == self.next_goal_y == 0:
                return self.FINISHED, (0,0)
        
        step = self.getNextStep()
        if step == self.NO_PATH:
            return self.getNextGoal(force_new_goal=True)
        return self.RUNNING, step


    def getFurthestTileFromStart(self):
        queue = Queue(maxsize=0)
        queue.put([0,0])
        visited = [[0,0]]
        previous = {}
        longestDistance = 0
        tileWithLongestDistance = [0,0]

        while not queue.empty():
            # get next Tile in queue
            processedTile = queue.get()
            # if tile is unvisited assign distance
            if self._map.getGroundState(processedTile[0], processedTile[1]) == Map.UNEXPLORED:
                lastTile = processedTile
                distance = 0
                # adds step as long as [0,0,0] not reached
                while not lastTile == [0,0]:
                    # increase distance
                    distance += 1
                    lastTile = previous[(lastTile[0],lastTile[1])]
                if distance > longestDistance:
                    longestDistance = distance
                    tileWithLongestDistance = processedTile
                # jump to next Tile in queue
                continue

            # add next tiles to queue
            for nextTile in self.getNeighbors(processedTile[0], processedTile[1]):
                # add next tile if not already processed
                if nextTile not in visited:
                    visited.append(nextTile)
                    queue.put(nextTile)
                    previous[(nextTile[0],nextTile[1])] = processedTile

        return tileWithLongestDistance[0], tileWithLongestDistance[1]

    def getNearestTileFromRobot(self):
        tile_x, tile_y = getCellCoords(self._rb.position_x, self._rb.position_y)

        queue = Queue(maxsize=0)
        queue.put([tile_x,tile_y])
        visited = [[tile_x,tile_y]]
        previous = {}
        shortestDistance = float("inf")
        tileWithShortestDistance = [0,0]

        while not queue.empty():
            # get next Tile in queue
            processedTile = queue.get()
            # if tile is unvisited assign distance
            if self._map.getGroundState(processedTile[0], processedTile[1]) == Map.UNEXPLORED:
                lastTile = processedTile
                distance = 0
                # adds step as long as [0,0,0] not reached
                while not lastTile == [tile_x,tile_y]:
                    # increase distance
                    distance += 1
                    lastTile = previous[(lastTile[0],lastTile[1])]
                if distance < shortestDistance:
                    shortestDistance = distance
                    tileWithShortestDistance = processedTile
                # jump to next Tile in queue
                continue

            # add next tiles to queue
            for nextTile in self.getNeighbors(processedTile[0], processedTile[1]):
                # add next tile if not already processed
                if nextTile not in visited:
                    visited.append(nextTile)
                    queue.put(nextTile)
                    previous[(nextTile[0],nextTile[1])] = processedTile

        return tileWithShortestDistance[0], tileWithShortestDistance[1]

    def getNextStep(self):
        #log("---Pathfinding---")
        tile_x, tile_y = getCellCoords(self._rb.position_x, self._rb.position_y)
        #log("Getting next step for tile:" + str(tile_x) + ", " + str(tile_y), " Goal: " + str(self.next_goal_x) + ", " + str(self.next_goal_y))
        queue = Queue(maxsize=0)
        queue.put([tile_x,tile_y])
        visited = [[tile_x,tile_y]]
        previous = {}
        while not queue.empty():
            # get next Tile in queue
            processedTile = queue.get()
            # log("Processing tile:" + str(processedTile[0]) + ", " + str(processedTile[1]))

            if processedTile == [self.next_goal_x, self.next_goal_y]:
                lastTile = processedTile
                second_last_tile = processedTile
                while not lastTile == [tile_x,tile_y]:
                    # increase distance
                    second_last_tile = lastTile
                    lastTile = previous[(lastTile[0],lastTile[1])]
                return second_last_tile
            if self._map.getGroundState(processedTile[0], processedTile[1]) == Map.UNEXPLORED:
                continue
            # add next tiles to queue
            # log("Neighbors: " + str(self.getNeighbors(processedTile[0], processedTile[1])))
            for nextTile in self.getNeighbors(processedTile[0], processedTile[1]):
                # add next tile if not already processed
                if nextTile not in visited:
                    # log(str(nextTile))
                    visited.append(nextTile)
                    queue.put(nextTile)
                    previous[(nextTile[0],nextTile[1])] = processedTile
        return 'NO_PATH'
    
    def getNeighbors(self, x, y):
        walls = self._map.getWalls(x,y)
        neighbors = []
        if walls[Map.NORTH] == Map.EMPTY:
            neighbors.append([x + 1, y])
        if walls[Map.EAST] == Map.EMPTY:
            neighbors.append([x, y + 1])
        if walls[Map.SOUTH] == Map.EMPTY:
            neighbors.append([x - 1, y])
        if walls[Map.WEST] == Map.EMPTY:
            neighbors.append([x, y - 1])
        
        return neighbors

class Dijkstras:
    
    # COSTS IN MS
    FORWARD_COST = 115
    FORWARD_COST_SWAMP = 300
    TURN_COST = 45
    TURN_COST_SWAMP = 150

    NO_PATH = -1
    FINISHED = 0
    RUNNING = 1

    def __init__(self, robot, map) -> None:
        self._rb = robot
        self._map = map.map
        

        self.last_goal_x = 0
        self.last_goal_y = 0
        self.path = []

        self.heap = Heap((self._map.map_size,self._map.map_size))

    def getNextGoal(self, force_new_goal = False):

        tile_x, tile_y = getCellCoords(self._rb.position_x, self._rb.position_y)

        # calculate the next tile to pathfind to if the current goal is reached, the next goal couldn't be reached or a recalculation is forced
        if len(self.path) == 0 or  not (tile_x == self.last_goal_x and tile_y == self.last_goal_y) or force_new_goal:

            self.calculatePath()
            # if no more goal is found and the robot is standing on the start tile, the run is ended
            if len(self.path) == 0:
                return self.FINISHED, (0,0)

        # get next step
        step = self.path.pop(0)
        # set last goal
        self.last_goal_x, self.last_goal_y = step
        return self.RUNNING, step

    def calculatePath(self):
        tile_x, tile_y = getCellCoords(self._rb.position_x, self._rb.position_y)

        self.heap.reset()
        self.heap.addTile(tile_x,tile_y,10000)
        visited = [[tile_x,tile_y]]
        previous = {}
        shortestDistance = float("inf")
        tileWithShortestDistance = [0,0]

        while not self.heap.isEmpty():
            # get next Tile in queue
            processedTileX, processedTileY, distance = self.heap.getFirstTile()
            # if tile is unvisited calculate distance
            if self._map.getGroundState(processedTileX, processedTileY) == Map.UNEXPLORED:
                if distance < shortestDistance:
                    shortestDistance = distance
                    tileWithShortestDistance = [processedTileX,processedTileY]
                # jump to next Tile in queue because unexplored can't have neighbours
                continue
            
            # set current orientation for use in cost calculation instead of previous tile
            currentOrientation = getDirection(self._rb.rotation)
            previous_tile = None
            if not(tile_x == processedTileX and tile_y == processedTileY):
                currentOrientation = None
                previous_tile = previous[(processedTileX, processedTileY)]

            # add next tiles to queue
            for nextTileX, nextTileY in self.getNeighbors(processedTileX, processedTileY):
                # add next tile if not already processed
                if [nextTileX, nextTileY] in visited:
                    continue
                
                # calculate Cost
                travelCost = self.getCost(previous_tile,[processedTileX,processedTileY], [nextTileX,nextTileY], currentOrientation)

                visited.append([nextTileX, nextTileY])

                self.heap.addTile(nextTileX,nextTileY, distance+travelCost)
                previous[(nextTileX,nextTileY)] = [processedTileX,processedTileY]
        
        # reconstruct path
        path = []
        x = tileWithShortestDistance[0]
        y = tileWithShortestDistance[1]
        while True:
            if x == tile_x and y == tile_y:
                break
            path.append((x,y))
            x,y = previous[(x,y)]

        path.reverse()
        self.path = path    

    def getCost(self,previous_tile, current_tile, next_tile, current_orient):
        # calculate current orientation
        if previous_tile != None:
            current_orient = self.getOrientation(previous_tile, current_tile)
        
        # calculate orientation after driving to next_tile
        next_orient = self.getOrientation(current_tile, next_tile)

        is_current_swamp = self._map.getGroundState(current_tile[0], current_tile[1]) == Map.SWAMP
        is_next_swamp = self._map.getGroundState(next_tile[0], next_tile[1]) == Map.SWAMP

        # calculate turn cost
        turn_cost = abs(next_orient-current_orient)
        if turn_cost > 2: turn_cost - 2
        
        if is_current_swamp: turn_cost *= self.TURN_COST_SWAMP
        # calculate forward cost by adding half the cost of both tiles together
        forward_cost = ([self.FORWARD_COST, self.FORWARD_COST_SWAMP][is_current_swamp] + [self.FORWARD_COST, self.FORWARD_COST_SWAMP][is_next_swamp])/2

        return turn_cost + forward_cost

    def getOrientation(self, from_tile, to_tile):
        if from_tile[0] < to_tile[0]:
            return 0
        if from_tile[1] < to_tile[1]:
            return 1
        if from_tile[0] > to_tile[0]:
            return 2
        if from_tile[1] > to_tile[1]:
            return 3

    def getNeighbors(self, x, y):
        walls = self._map.getWalls(x,y)
        neighbors = []
        if walls[Map.NORTH] == Map.EMPTY:
            neighbors.append([x + 1, y])
        if walls[Map.EAST] == Map.EMPTY:
            neighbors.append([x, y + 1])
        if walls[Map.SOUTH] == Map.EMPTY:
            neighbors.append([x - 1, y])
        if walls[Map.WEST] == Map.EMPTY:
            neighbors.append([x, y - 1])
        
        return neighbors
