import cv2
import numpy as np

# load and show an image with Pillow
from PIL import Image

def sort_points(pts):
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

def filterContours(contours):
    newContours = []
    for i, contour in enumerate(contours):
        if cv2.contourArea(contour) < 200:
            continue
        if len(contour) < 4:
            continue
        
        inside = False
        # check if there is a contour inside the contour
        for j, testcontour in enumerate(contours):
            # check if contour is the same
            if j == i:
                continue
            if(cv2.pointPolygonTest(contour,tuple(testcontour[0][0]), False) >= 0):
                print(cv2.contourArea(contour))
                newContours.append(contour)
    return np.array(newContours)


# Open the image form working directory

image = Image.open('detected/detected_5.png')
#image = Image.open('detected/detected_11.png')
#image = Image.open('front/front_10.png')

img = np.copy(np.asarray(image))
original = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
##########
height = int(original.shape[0])
width = int(original.shape[1])
# convert to grayscale and crop

#crop image
#cropped = original[int(height/3):height - int(height/3),int(width/5):width-int(width/5),:]
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

contours = filterContours(contours)


if len(contours) < 0:
    exit()

################ find correct lettern type
### get rectangle alone
rect_contour = contours[0]
# approximate rextangle
rect_contour = cv2.approxPolyDP(rect_contour,0.04*cv2.arcLength(rect_contour,True), True)
# check id contour has 4 points
if len(rect_contour) != 4:
    exit()

# order rectangle points
rect_contour = sort_points(np.float32([list(r[0]) for r in rect_contour]))
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
if top_point and middle_point and lowest_point:
    print("S")
elif middle_point:
    print("H")
elif lowest_point:
    print("U")
########################

display_array = cv2.cvtColor(letter_image, cv2.COLOR_GRAY2RGB)
display_array = cv2.circle(display_array, (int(w/2),int(h/10)), radius=0, color=(0, 255, 0), thickness=-1)
display_array = cv2.circle(display_array, (int(w/2),int(h/10*5)), radius=0, color=(0, 255, 0), thickness=-1)
display_array = cv2.circle(display_array, (int(w/2),int(h/10*9)), radius=0, color=(0, 255, 0), thickness=-1)

#cv2.drawContours(display_array, contour_of_letter, -1, (0,255,0), 1)
display_image = Image.fromarray(display_array)
display_image.show()
### identify thing
#middleRow = dst[:,50:51]
#print("Mean:", middleRow.mean())
#display_array = cv2.cvtColor(middleRow, cv2.COLOR_GRAY2RGB)
#display_image = Image.fromarray(display_array)
#display_image.show()
##########
#display_array = cv2.cvtColor(thresh, cv2.COLOR_GRAY2RGB)

#cv2.drawContours(display_array, contours, -1, (0,255,0), 1)
#display_image = Image.fromarray(display_array)
#display_image.show()