from victimHandler import VictimHandler
from Pathfinding import BreadthFirstSearch, Dijkstras
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
pf = Dijkstras(robot, mp)
vh = VictimHandler(robot, mp, dc, cp)
   
dc.setGoal(0,0)
scannedNextTile = False
scannedForVictimSecondTime = False

def getNextGoal(force_recalculation=False):
    # plan path
    status, goal = pf.getNextGoal(force_recalculation)

    # if path planner doesn't find a goal return "FINISHED"
    if status == pf.RUNNING:
        dc.setGoal(goal[0]* TILESIZE, goal[1]* TILESIZE)
    # set goal in drivemanager
    return status

# main loop of the simulation
while robot.update() == robot.PROGRAM_RUNNING:
    if robot.LoP:
        robot.LoP = False
        dc.correctRotation()
        getNextGoal(True)
    # get new goal if current goal is reached
    if dc.update() == dc.GOAL_REACHED:
        # update map
        mp.processTile()

        # process detected victims
        vh.updateOnGoalReached()
        
        scannedNextTile = False
        scannedForVictimSecondTime = False

        # get next goal or exit maze if finished
        if getNextGoal() == pf.FINISHED:
            robot.exit_maze()

    # search for victims a second time if ca. in middle of tile
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