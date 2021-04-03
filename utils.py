import math
DEBUG = True
TILESIZE = 0.12
def log(message, data=None):
    if DEBUG:
        if data is None:
            print(message)
        else:
            print(message + str(data))

def calculateEucleadianDistance(x1, y1, x2, y2):
    return ((x1-x2)**2+(y1-y2)**2)**0.5

def getCellCoords(x,y):
    return (int((x+TILESIZE/2*(x/abs(x)))/TILESIZE),int((y+(TILESIZE/2*(y/abs(y))))/TILESIZE))    

def getDirection(rotation):
    direction = int((rotation + math.radians(225)) / (math.pi/2)) + 2
    if direction > 3: direction -= 4
    return direction
