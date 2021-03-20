import numpy as np
import cv2
from utils import *
try:
    from PIL import Image
except:
    pass
class CameraProcessor:

    MEAN_TRESHOLD_FOR_BLACK_TILE = 30
    MEAN_TRESHOLD_FOR_OBSTACLE = 250

    # Threshold of sky in HSV space
    lower_blue = np.array([10, 0, 0])
    upper_blue = np.array([255, 255, 255])

    def __init__(self, robot) -> None:

        self._rb = robot
        self.front_counter = 0
        self.left_counter = 0
        self.right_counter = 0

    def saveImages(self):
        # front 
        camera = self._rb._front_camera
        original = np.array(np.frombuffer(self._rb.front_camera_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
        original = cv2.cvtColor(original, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(original)
        img.save("front/front_" + str(self.front_counter) +  ".png" , "PNG")
        self.front_counter += 1

        # left 
        camera = self._rb._left_camera
        original = np.array(np.frombuffer(self._rb.left_camera_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
        original = cv2.cvtColor(original, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(original)
        img.save("left/left_" + str(self.left_counter) +  ".png" , "PNG")
        self.left_counter += 1
        # right
        camera = self._rb._right_camera
        original = np.array(np.frombuffer(self._rb.right_camera_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
        original = cv2.cvtColor(original, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(original)
        img.save("right/right_" + str(self.right_counter) +  ".png" , "PNG")
        self.right_counter += 1


    def searchForHole(self, original):
        previous_height = original.shape[0]
        new_height = int(previous_height/4)

        img = original[previous_height-new_height:previous_height,:,:]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        mean = gray.mean()

        log("Hole mean: " + str(mean))

        if mean < self.MEAN_TRESHOLD_FOR_BLACK_TILE:
            log("Hole detected")
            return True
        return False

    def searchForObstacle(self, original):
        # self.saveImages()
        new_height = int(original.shape[0]/4)
        padding_right_left = int(original.shape[0] * 0.0)

        img = original[int(new_height/2):new_height,padding_right_left:original.shape[0]-padding_right_left,:]

        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(img, self.lower_blue, self.upper_blue)

        log("Obstacle mean: ", str(mask.mean()))
        


        if mask.mean() < self.MEAN_TRESHOLD_FOR_OBSTACLE:
            # x = cv2.cvtColor(original, cv2.COLOR_BGR2RGB)
            # image2 = Image.fromarray(x)
            # image2.show()
            log("Obstacle detected")
            return True
        return False

    def isNextTileImpassable(self):
        # get image
        camera = self._rb._front_camera
        original = np.array(np.frombuffer(self._rb.front_camera_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
        
        if self.searchForHole(original) or self.searchForObstacle(original):
            return True
        return False


