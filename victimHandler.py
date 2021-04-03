from numpy import tile
from mapper import Map
from utils import *
import math
class VictimHandler:

    def __init__(self, robot, mapper, driveController, cameraProcessor) -> None:
        self._rb = robot
        self._map = mapper.map
        self._dc = driveController
        self._cp = cameraProcessor
        
        self.victimOnNextTile = False
        self.victimType = None
        self.tilesWithVisual = []

    def update(self):
        # update heat victim
        tile_x, tile_y = getCellCoords(self._rb.position_x, self._rb.position_y)
        direction = getDirection(self._rb.rotation)

        if self._rb.is_victim_right:
            if self._map.getWalls(tile_x, tile_y)[direction-3] == Map.WALL:
                self._rb.reportVictim(self._rb.HEATED_VICTIM)
                self._dc.standStill(3)
        elif self._rb.is_victim_left:
            if self._map.getWalls(tile_x, tile_y)[direction-1] == Map.WALL:
                self._rb.reportVictim(self._rb.HEATED_VICTIM)
                self._dc.standStill(3)

    def updateOnIsFacingGoal(self):
        right = self._cp.scanRight()
        if right is not None: 
            self.victimType = right           
            self.victimOnNextTile = True

        left = self._cp.scanLeft()
        if left is not None: 
            self.victimType = left           
            self.victimOnNextTile = True
    
    def updateOnGoalReached(self):
        tile_x, tile_y = getCellCoords(self._rb.position_x, self._rb.position_y)
        if self.victimOnNextTile and not (tile_x,tile_y) in self.tilesWithVisual:
            log(f"Visual victim {self.victimType} on this tile")
            self._rb.reportVictim(self.victimType)
            self._dc.standStill(3)
            self.tilesWithVisual.append((tile_x,tile_y))

        self.victimOnNextTile = False

    def updateOnEnteredNewTile(self):
        right = self._cp.scanRight()
        if right is not None: 
            self.victimType = right           
            self.victimOnNextTile = True

        left = self._cp.scanLeft()
        if left is not None: 
            self.victimType = left           
            self.victimOnNextTile = True
    
        front = self._cp.scanFront()
        if front is not None: 
            self.victimType = front           
            self.victimOnNextTile = True
    

