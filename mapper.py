from utils import *
import numpy as np

MAP_SIZE = 50

MAP_LOGGING = False
def logMap(map):
    if MAP_LOGGING:
        output = "\u001b[37m"
        
        # first line
        for row in range(map.map_size):
            if map.getWallsWithoutTileOffset(map.map_size-1,row)[Map.NORTH]:
                output += "\u001b[31m --\u001b[37m"
            else:
                output += " --"
        output += "\n"
        # other lines
        for line in range(map.map_size * 2 - 1, -1, -1):
            # first row
            if line % 2 == 1:
                if map.getWallsWithoutTileOffset(int(line / 2),0)[Map.WEST]:
                    output += "\u001b[31m|\u001b[37m"
                else:
                    output += "|"
            # other rows
            for row in range(map.map_size):
                # horizontal walls
                if line % 2 == 0:
                    if map.getWallsWithoutTileOffset(int(line / 2),row)[Map.SOUTH]:
                        output += " \u001b[31m--\u001b[37m"
                    else:
                        output += " --"
                # vertical walls
                else:
                    if map.getGroundStateWithoutTileOffset(int(line / 2),row) == Map.NORMAL:
                        output += "\u001b[47;31m  \u001b[0m\u001b[37m"
                    elif map.getGroundStateWithoutTileOffset(int(line / 2),row) == Map.HOLE:
                        output += "\u001b[41;31m  \u001b[0m\u001b[37m"

                    else:
                        output += "  "
                    if map.getWallsWithoutTileOffset(int(line / 2),row)[Map.EAST]:
                        output += "\u001b[31m|\u001b[37m"
                    else:
                        output += "|"
                    
            output += "\n"
        output += "\u001b[0m"
        print(output)
        

        

class Mapper:

    WALL_DISTANCE_TRESHOLD = 0.1

    def __init__(self, robot) -> None:
        self._rb = robot
        self.map = Map()
        self.victimList = []

    def processTile(self):
        tile_x, tile_y = getCellCoords(self._rb.position_x, self._rb.position_y)
        #log("Process Tile:" + str(tile_x) + ", " + str(tile_y))

        walls = np.zeros(4,int)
        walls[0] = int(self._rb.distance_sensor_values[0]<self.WALL_DISTANCE_TRESHOLD)
        walls[1] = int(self._rb.distance_sensor_values[2]<self.WALL_DISTANCE_TRESHOLD)
        walls[2] = Map.OLD_STATE
        walls[3] = int(self._rb.distance_sensor_values[5]<self.WALL_DISTANCE_TRESHOLD)
        
        # rotate walls to always have the same orientation on the map
        walls = np.roll(walls, getDirection(self._rb.rotation))
        
        self.map.setWalls(tile_x, tile_y, walls[0], walls[1], walls[2], walls[3])
        self.map.setGroundState(tile_x, tile_y, Map.NORMAL)
        logMap(self.map)
        

    def blackTileIsAhead(self):
        tile_x, tile_y = getCellCoords(self._rb.position_x, self._rb.position_y)


        direction = getDirection(self._rb.rotation)

        if direction == 0:
            tile_x += 1
        elif direction == 1:
            tile_y += 1
        elif direction == 2:
            tile_x -= 1
        elif direction == 3:
            tile_y -= 1

        self.map.setGroundState(tile_x, tile_y, Map.HOLE)
        self.map.setWalls(tile_x, tile_y, True, True, True, True)

        logMap(self.map)

class Map:
    # indices of fields of tile
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3
    TILE_STATE = 4

    # wall states
    OLD_STATE = -1
    EMPTY = 0
    WALL = 1

    
    # tile_state
    UNEXPLORED = 0
    NORMAL = 1
    SWAMP = 2
    HOLE = 3

    
    
    def __init__(self) -> None:
        self.map_size = MAP_SIZE
        self.map = np.zeros((MAP_SIZE,MAP_SIZE,5), dtype=int)
        self.tile_offset = int(MAP_SIZE / 2)
        self.setWalls(0, 0, self.WALL, self.WALL, self.WALL, self.WALL)

    def getTile(self, x, y):
        try:
            if not (x + self.tile_offset<0 or y + self.tile_offset<0):        
                return self.map[x + self.tile_offset, y + self.tile_offset]
        except:
            pass    
        return np.array([1,1,1,1,1])

    def setWalls(self, x, y, north, east, south, west):

        # if wall is None replace with current state of wall:
        if north == -1:
            north = self.getWalls(x,y)[self.NORTH]

        if east == -1:
            east = self.getWalls(x,y)[self.EAST]

        if south == -1:
            south = self.getWalls(x,y)[self.SOUTH]

        if west == -1:
            west = self.getWalls(x,y)[self.WEST]

        tile = self.getTile(x, y)

        if self.getGroundState(x + 1, y) != self.HOLE:
            north_tile = self.getTile(x + 1, y)
            tile[self.NORTH] = int(north)
            north_tile[self.SOUTH] = int(north)

        if self.getGroundState(x, y + 1) != self.HOLE:
            east_tile = self.getTile(x, y +1)
            tile[self.EAST] = int(east)
            east_tile[self.WEST] = int(east)

        if self.getGroundState(x - 1, y) != self.HOLE:
            south_tile = self.getTile(x - 1, y)
            tile[self.SOUTH] = int(south)
            south_tile[self.NORTH] = int(south)

        if self.getGroundState(x, y - 1) != self.HOLE:
            west_tile = self.getTile(x, y - 1)
            tile[self.WEST] = int(west)
            west_tile[self.EAST] = int(west)

    def setGroundState(self, x, y, groundState):
        tile = self.getTile(x, y)
        tile[self.TILE_STATE] = int(groundState)

    def getWalls(self, x, y):
        tile = self.getTile(x, y)
        return (tile[self.NORTH], tile[self.EAST], tile[self.SOUTH], tile[self.WEST])

    def getWallsWithoutTileOffset(self, x, y):
        tile = self.map[x,y]
        return (tile[self.NORTH], tile[self.EAST], tile[self.SOUTH], tile[self.WEST])

    def getGroundState(self, x, y):
        tile = self.getTile(x, y)
        return tile[self.TILE_STATE]

    def getGroundStateWithoutTileOffset(self, x, y):
        tile = self.map[x,y]
        return tile[self.TILE_STATE]