from victimHandler import VictimHandler
from Pathfinding import Pathfinder
from numpy import tile
from robot import *
from utils import *
from DriveController import * 
from mapper import *
from camera import *

robot = MazeRobot()
dc = DriveController(robot)
mp = Mapper(robot)
cp = CameraProcessor(robot)
pf = Pathfinder(robot, mp)
vh = VictimHandler(robot, mp, dc, cp)

dc.correctRotation()
    
dc.setGoal(0,0)
scannedNextTile = False
scannedForVictimSecondTime = False

def getNextGoal(force_recalculation=False):
    goal = pf.getNextGoal()
    if goal == "FINISHED":
        return True
    dc.setGoal(goal[0]* TILESIZE, goal[1]* TILESIZE)
    return False

while robot.update():

    # get new goal if current goal is reached
    if dc.update():
        mp.processTile()
        vh.updateOnGoalReached()
        scannedNextTile = False
        scannedForVictimSecondTime = False
        # exit maze if finished
        if getNextGoal():
            robot.exit_maze()

    if dc.onNewTile() and not scannedForVictimSecondTime:
        scannedForVictimSecondTime = True
        vh.updateOnEnteredNewTile()


    # scan tile for black tile 
    if dc.isFacingGoal() and not scannedNextTile:
        log("----")
        vh.updateOnIsFacingGoal()

        if cp.isNextTileImpassable():
            mp.blackTileIsAhead()
            getNextGoal(True)
            continue
        scannedNextTile = True
        log("----")
    vh.update()

log("stop Program")