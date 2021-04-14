from utils import *
import math

class DriveController():

    GOAL_REACHED = 1
    DRIVING_TO_GOAL = 0
    DRIVING_PRECISION = 0.005
    MAX_SPEED = 6.28
    CORRECT_ROTATION_RATE = 10
    # PID controller consts
    Kp_angle = 4
    Kp_distance = 100
    
    min_error_for_driving_forward = math.pi/ 32

    def __init__(self, robot) -> None:
        self._rb = robot
        self._goal_x = robot.position_x
        self._goal_y = robot.position_y
        self.direction = 0
        self.goal_counter_for_correcting_rotation = 0

    def setGoal(self, x, y, direction=None):
        # direction = 1 if in y direction and direction = 0 if in x direction
        self.direction = int(abs(self._rb.position_x - x) < abs(self._rb.position_y - y))
        self._goal_x = x
        self._goal_y = y
        self.goal_counter_for_correcting_rotation += 1
        

    def update(self):

        distance_error = calculateEucleadianDistance(self._goal_x, self._goal_y, self._rb.position_x, self._rb.position_y)
        
        if self.goal_counter_for_correcting_rotation >= self.CORRECT_ROTATION_RATE:
            self.correctRotation()


        if self.isGoalReached():
            self._rb.setSpeed(0,0)
            return self.GOAL_REACHED
        
        # calculate goal angle error
        angle_error = self.calculateAngleError()

        # pid controllerls

        angle_p_value = self.Kp_angle * angle_error
        angle_p_value = max(-1, min(1, angle_p_value))

        distance_p_value = self.Kp_distance * distance_error
        distance_p_value = max(-1, min(1, distance_p_value))
        #distance_p_value = 1
        
        angle_importance = 1

        if abs(angle_error) < self.min_error_for_driving_forward:
            angle_importance = abs(angle_error) / self.min_error_for_driving_forward

        distance_importance = 1 - abs(angle_importance)  
            
        # apply pid controller to speed
        self._rb.setSpeed((angle_importance * -angle_p_value + distance_importance * distance_p_value) * self.MAX_SPEED,
                          (angle_importance *  angle_p_value + distance_importance * distance_p_value) * self.MAX_SPEED)
        return "False"

    def calculateAngleError(self):
        distances_to_goal = [self._goal_x - self._rb.position_x, self._goal_y - self._rb.position_y]
        
        if 0 < distances_to_goal[self.direction] < 0.05:
            distances_to_goal[self.direction] = 0.05
        if -0.05 < distances_to_goal[self.direction] < 0:
            distances_to_goal[self.direction] = -0.05
        

        angle_to_goal = math.atan2(
            distances_to_goal[1],
            distances_to_goal[0],
        )
        angle_error = angle_to_goal -  self._rb.rotation
        if angle_error > math.pi: angle_error -= 2 * math.pi
        if angle_error < -math.pi:angle_error += 2 * math.pi
        return angle_error

    def correctRotation(self):
        self.goal_counter_for_correcting_rotation = 0
        self._rb.setSpeed(3,3)
        self._rb.sleep(0.2)
        self._rb.setSpeed(-3,-3)
        self._rb.sleep(0.2)
        
    def isGoalReached(self):
        distances = [self._goal_x - self._rb.position_x, self._goal_y - self._rb.position_y]

        if abs(distances[self.direction]) < self.DRIVING_PRECISION:
            return True
        return False        

    def isFacingGoal(self):
        angle_error = self.calculateAngleError()
        if abs(angle_error) < math.pi / 32:
            return True
        return False

    def getDistanceFromGoal(self):
        return calculateEucleadianDistance(self._goal_x, self._goal_y, self._rb.position_x, self._rb.position_y)


    def standStill(self, seconds):
        self._rb.setSpeed(0,0)
        self._rb.sleep(seconds)
