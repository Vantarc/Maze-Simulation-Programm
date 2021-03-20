
from utils import *
from mapper import Map
from queue import Queue

class Pathfinder:

    def __init__(self, robot, map) -> None:
        self._rb = robot
        self._map = map.map
        self.next_goal_x = 0
        self.next_goal_y = 0

    def getNextGoal(self, force_new_goal = False):
        tile_x, tile_y = getCellCoords(self._rb.position_x, self._rb.position_y)


        if tile_x == self.next_goal_x and tile_y == self.next_goal_y or self._map.getGroundState(self.next_goal_x,self.next_goal_y) == Map.HOLE or force_new_goal:
            self.next_goal_x, self.next_goal_y = self.getNearestTileFromRobot()
            if tile_x == self.next_goal_x == 0 and tile_y == self.next_goal_y == 0:
                return "FINISHED"
        
        step = self.getNextStep()
        if step == 'NO_PATH':
            return self.getNextGoal(force_new_goal=True)
        return step


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