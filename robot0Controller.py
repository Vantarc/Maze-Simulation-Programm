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
    # plan path
    goal = pf.getNextGoal()

    # if path planner doesn't find a goal return "FINISHED"
    if goal == "FINISHED":
        return goal
    
    # set goal in drivemanager
    dc.setGoal(goal[0]* TILESIZE, goal[1]* TILESIZE)
    return "RUNNING"

# main loop of the simulation
while robot.update() == robot.PROGRAM_RUNNING:

    # get new goal if current goal is reached
    if dc.update() == dc.GOAL_REACHED:
        # update map
        mp.processTile()

        # process detected victims
        vh.updateOnGoalReached()
        
        scannedNextTile = False
        scannedForVictimSecondTime = False

        # get next goal or exit maze if finished
        if getNextGoal() == "FINISHED":
            robot.exit_maze()

    # search for victims a second time
    if dc.getDistanceFromGoal() < TILESIZE/1.5 and not scannedForVictimSecondTime:
        scannedForVictimSecondTime = True
        vh.updateOnEnteredNewTile()
        cp.saveImages()

    # scan tile for black tile and for victims if robot is facing to the goal for the first time
    if dc.isFacingGoal() and not scannedNextTile:
        vh.updateOnIsFacingGoal()
        cp.saveImages()
        
        # recalculate path if next tile is impassable
        if cp.isNextTileImpassable():
            mp.blackTileIsAhead()
            getNextGoal(True)
            continue
        scannedNextTile = True
    vh.update()

log("stop Program")