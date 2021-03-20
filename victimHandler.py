from mapper import Map
from utils import *
import math
class VictimHandler:

    def __init__(self, robot, mapper, driveController, cameraProcessor) -> None:
        self._rb = robot
        self._map = mapper.map
        self._dc = driveController
        self._cp = cameraProcessor

    def update(self):
        # update heat victim
        tile_x, tile_y = getCellCoords(self._rb.position_x, self._rb.position_y)
        direction = getDirection(self._rb.rotation)

        if self._rb.is_victim_right:
            if self._map.getWalls(tile_x, tile_y)[direction-3] == Map.WALL:
                self._dc.standStill(1)
                self._rb.reportVictim(self._rb.HEATED_VICTIM)
                self._dc.standStill(2)
        elif self._rb.is_victim_left:
            if self._map.getWalls(tile_x, tile_y)[direction-1] == Map.WALL:
                self._dc.standStill(1)
                self._rb.reportVictim(self._rb.HEATED_VICTIM)
                self._dc.standStill(2)

    def updateOnisFacingGoal():
        pass