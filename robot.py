from controller import Robot
import math
import struct
from utils import *

class MazeRobot():

    TEMPERATURE_TRESHOLD = 27

    HARMED_VICTIM = 'H'
    STABLE_VICTIM = 'S'
    UNHARMED_VICTIM = 'U'
    HEATED_VICTIM = 'T'

    PROGRAM_EXITED = 0
    PROGRAM_RUNNING = 1

    def __init__(self) -> None:
        # Simulation time step and the maximum velocity of the robot
        self.timeStep = 32
        
        self._robot = Robot()

        self._initialize_devices()

        self._initialize_variables()

        self.update()
        self._position_offset_x = self.position_x
        self._position_offset_y = self.position_y

        
    def _initialize_devices(self) -> None:
        # Declare motors/wheels
        self._wheel_left = self._robot.getMotor("left wheel motor")
        self._wheel_right = self._robot.getMotor("right wheel motor")
        self._wheel_left.setPosition(float("inf"))
        self._wheel_right.setPosition(float("inf"))
        # Declare cameras
        self._front_camera = self._robot.getCamera("camera_centre")
        self._front_camera.enable(self.timeStep)
        
        self._right_camera = self._robot.getCamera("camera_right")
        self._right_camera.enable(self.timeStep)
        
        self._left_camera = self._robot.getCamera("camera_left")
        self._left_camera.enable(self.timeStep)

        # Declare colour sensor underneath the robot
        self._colour_sensor = self._robot.getCamera("colour_sensor")
        self._colour_sensor.enable(self.timeStep)

        # Declare communication link between the robot and the controller
        self._emitter = self._robot.getEmitter("emitter")

        # Declare GPS and gyro
        self._gps = self._robot.getGPS("gps")
        self._gps.enable(self.timeStep)

        self._gyro = self._robot.getGyro("gyro")
        self._gyro.enable(self.timeStep)
        # Declare heat/temperature sensor
        self._left_heat_sensor = self._robot.getLightSensor("left_heat_sensor")
        self._left_heat_sensor.enable(self.timeStep)

        self._right_heat_sensor = self._robot.getLightSensor("right_heat_sensor")
        self._right_heat_sensor.enable(self.timeStep)

        # Declare distance sensors
        self._ds0 = self._robot.getDistanceSensor("ps0")
        self._ds0.enable(self.timeStep)

        self._ds1 = self._robot.getDistanceSensor("ps1")
        self._ds1.enable(self.timeStep)

        self._ds2 = self._robot.getDistanceSensor("ps2")
        self._ds2.enable(self.timeStep)

        self._ds3 = self._robot.getDistanceSensor("ps3")
        self._ds3.enable(self.timeStep)

        self._ds4 = self._robot.getDistanceSensor("ps4")
        self._ds4.enable(self.timeStep)

        self._ds5 = self._robot.getDistanceSensor("ps5")
        self._ds5.enable(self.timeStep)

        self._ds6 = self._robot.getDistanceSensor("ps6")
        self._ds6.enable(self.timeStep)

        self._ds7 = self._robot.getDistanceSensor("ps7")
        self._ds7.enable(self.timeStep)

        # Declare communication link between the robot and the controller
        self._emitter = self._robot.getEmitter("emitter")

    def _initialize_variables(self):
        self.LoP = False
        self.max_velocity = 6.28
        # [x,y,euclidean, rot]
        self.robot_velocity = [0,0,0,0]
        # speed and position
        self._speed_right = 0
        self._speed_left = 0

        self.position_x = 0
        self.position_y = 0

        self._last_position_x = 0
        self._last_position_y = 0
        self._last_rotation = 0
        self._position_offset_x = 0
        self._position_offset_y = 0
        
        self.rotation = 0

        # sensor values
        self.is_victim_right = False
        self.is_victim_left = False
        
        self.distance_sensor_values = [0]*8

        self.ground_color = None
        self.right_camera_data = None
        self.front_camera_data = None
        self.left_camera_data = None

    def setSpeed(self,right_speed, left_speed):
        self._speed_right, self._speed_left = right_speed, left_speed
        
    def update(self):
        # update Velocity
        self._wheel_left.setVelocity(self._speed_left)
        self._wheel_right.setVelocity(self._speed_right)

        if self._robot.step(self.timeStep) == -1:
            return self.PROGRAM_EXITED
        
        # save rotation and position from last frame
        self._last_position_x = self.position_x
        self._last_position_y = self.position_y
        self._last_rotation = self.rotation

        # set new position
        gps_values = self._gps.getValues()
        self.position_x = gps_values[0] - self._position_offset_x
        self.position_y = gps_values[2] - self._position_offset_y

        # calculate new rotation
        self.rotation -= self._gyro.getValues()[0] * (self.timeStep / 1000)
        if   self.rotation >  math.pi: self.rotation -= 2 * math.pi
        elif self.rotation < -math.pi: self.rotation += 2 * math.pi

        # correct rotation if possible
        if self._speed_left == self._speed_right and self._speed_right > 0:
            calculated_rotation_by_gps = math.atan2(self.position_y - self._last_position_y,
                                                    self.position_x - self._last_position_x)
            self.rotation = calculated_rotation_by_gps

        # calculate velocity
        self.robot_velocity[0] = self.position_x - self._last_position_x
        self.robot_velocity[1] = self.position_y - self._last_position_y
        self.robot_velocity[2] = calculateEucleadianDistance(self.position_x, self.position_y, self._last_position_x, self._last_position_y)
        self.robot_velocity[3] = self.rotation - self._last_rotation

        # check for LoP
        if self.robot_velocity[2]>0.01:
            self.LoP = True
        # sensor values
        self.is_victim_right = self._right_heat_sensor.getValue() > self.TEMPERATURE_TRESHOLD
        self.is_victim_left = self._left_heat_sensor.getValue() > self.TEMPERATURE_TRESHOLD

        self.distance_sensor_values[0] = self._ds0.getValue()
        self.distance_sensor_values[1] = self._ds1.getValue()
        self.distance_sensor_values[2] = self._ds2.getValue()
        self.distance_sensor_values[3] = self._ds3.getValue()
        self.distance_sensor_values[4] = self._ds4.getValue()
        self.distance_sensor_values[5] = self._ds5.getValue()
        self.distance_sensor_values[6] = self._ds6.getValue()
        self.distance_sensor_values[7] = self._ds7.getValue()
 
        self.ground_color = self._colour_sensor.getImage()
        self.right_camera_data = self._right_camera.getImage()
        self.front_camera_data = self._front_camera.getImage()
        self.left_camera_data = self._left_camera.getImage()
        return self.PROGRAM_RUNNING

    def reportVictim(self, type):
        log("Report Victim: ", type)
        position = self._gps.getValues()
        message = struct.pack('i i c', int(position[0] * 100), int(position[2] * 100), type.encode())
        self._emitter.send(message)

    def exit_maze(self):
        log("Get Exit bonus")
        message = struct.pack('i i c', 0, 0, 'E'.encode())
        self._emitter.send(message)
    
    def sleep(self, seconds):
        start_time = self._robot.getTime()
        while(self._robot.getTime() - start_time < seconds):
            self.update()