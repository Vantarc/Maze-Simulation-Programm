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

    SIZE_TRESHOLD_FOR_VICTIM = 155

    # Threshold of sky in HSV space
    lower_blue = np.array([10, 0, 0])
    upper_blue = np.array([255, 255, 255])

    def __init__(self, robot) -> None:

        self._rb = robot
        self.front_counter = 0
        self.left_counter = 0
        self.right_counter = 0
        self.detected_counter = 0

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


    def scanLeft(self):
        camera = self._rb._left_camera
        original = np.copy(np.array(np.frombuffer(self._rb.left_camera_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))))
        return self.searchForVictim(original,"LEFT")

    def scanFront(self):
        camera = self._rb._front_camera
        original = np.copy(np.array(np.frombuffer(self._rb.front_camera_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))))
        return self.searchForVictim(original,"FRONT")

    def scanRight(self):
        camera = self._rb._right_camera
        original = np.copy(np.array(np.frombuffer(self._rb.right_camera_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))))
        return self.searchForVictim(original,"RIGHT")

    def searchForVictim(self,original, cam_direction):
        save = np.copy(original)
        height = int(original.shape[0])

        # convert to grayscale and crop

        #crop image
        cropped = original[int(height/3):height - int(height/3),:,:]
        # filter red 
        cropped[:,:,2] = np.zeros([cropped.shape[0], cropped.shape[1]])

        # convert to grayscale
        gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
        # apply treshold
        ret, thresh = cv2.threshold(gray, 130, 255, cv2.THRESH_BINARY)
        # find contours
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # filter contours
        contours = self.filterContours(contours)
        if len(contours) == 0:
            return None

        ################ find correct lettern type
        ### get rectangle alone
        rect_contour = contours[0]
        # approximate rextangle
        rect_contour = cv2.approxPolyDP(rect_contour,0.04*cv2.arcLength(rect_contour,True), True)
        # check id contour has 4 points
        if len(rect_contour) != 4:
            return None

        # order rectangle points
        rect_contour = self.sort_points(np.float32([list(r[0]) for r in rect_contour]))
        # perspektive transform
        M = cv2.getPerspectiveTransform(rect_contour,np.float32([[0,0],[100,0],[100,100],[0,100]]))
        rect_image = cv2.warpPerspective(gray,M,(100,100))

        ### get letter alone
        # apply treshold
        ret, rect_image = cv2.threshold(rect_image, 130, 255, cv2.THRESH_BINARY_INV)
        # find contours
        contour_of_letter, hierarchy = cv2.findContours(rect_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # filter for contour of letter
        max_area = max([cv2.contourArea(c) for c in contour_of_letter])
        contour_of_letter = np.array([c for c in contour_of_letter if cv2.contourArea(c) == max_area][0],dtype="int32")

        # get bounding box of letter
        x,y,w,h = cv2.boundingRect(contour_of_letter)

        letter_image = rect_image[y:y+h,x:x+w]

        ### decide for letter
        top_point = letter_image[int(h/10),int(w/2)] == 255
        middle_point = letter_image[int(h/10*5),int(w/2)] == 255
        lowest_point = letter_image[int(h/10*9),int(w/2)] == 255
        print(cam_direction, self.front_counter)
        if top_point and middle_point and lowest_point:
            print("S")
            return self._rb.STABLE_VICTIM
        elif middle_point:
            print("H")
            return self._rb.HARMED_VICTIM
        elif lowest_point:
            print("U")
            return self._rb.UNHARMED_VICTIM
        return None
    
    def sort_points(self, pts):
        # sort the points based on their x-coordinates
        xSorted = pts[np.argsort(pts[:, 0]), :]

        # grab the left-most and right-most points from the sorted
        # x-xoodinate points
        leftMost = xSorted[:2, :]
        rightMost = xSorted[2:, :]

        # now, sort the left-most coordinates according to their
        # y-coordinates so we can grab the top-left and bottom-left
        # points, respectively
        leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
        (tl, bl) = leftMost
        # now that we have the top-left coordinate, use it as an
        # anchor to calculate the Euclidean distance between the
        # top-left and right-most points; by the Pythagorean
        # theorem, the point with the largest distance will be
        # our bottom-right point
        rightMost = rightMost[np.argsort(rightMost[:, 1]), :]
        (tr, br) = rightMost

        # return the coordinates in top-left, top-right,
        # bottom-right, and bottom-left order
        return np.array([tl, tr, br, bl], dtype="float32")

    def filterContours(self, contours):
        quadContours = []
        for i, contour in enumerate(contours):
            contourArea = cv2.contourArea(contour)
            if contourArea < self.SIZE_TRESHOLD_FOR_VICTIM:
                continue
            if len(contour) < 4:
                continue
            
            inside = False
            # check if there is a contour inside the contour
            for j, testcontour in enumerate(contours):
                # check if contour is the same
                if j == i:
                    continue
                # check if contour is smaller
                if not (contourArea*0.2 < cv2.contourArea(testcontour) < contourArea*0.9):
                    continue
                if(cv2.pointPolygonTest(contour,tuple(testcontour[0][0]), False) >= 0):
                    quadContours.append(contour)

        return np.array(quadContours)
