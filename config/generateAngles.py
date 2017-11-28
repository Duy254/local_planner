#code from: https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python/13849249#13849249
import numpy as np
import sys
import csv
from math import *

isDeg = False

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)

     #return degrees if user entered "d" as 2nd argument
    if len(sys.argv) > 3 and sys.argv[3] == 'd':
        global isDeg
        isDeg = True
        angleBetw = np.rad2deg(np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)))
        if np.dot(v1, v2) < 0:
            angleBetw *= -1
        return angleBetw

    else: #return radians
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def constrain(angle, lowBound, hiBound):
    while angle < lowBound:
        angle += 2 * pi
    while angle > hiBound:
        angle -= 2 * pi
    return angle

tupleList = []
coordCsv = [] #list of coordinates read in from file

#open list of coordinates
if (len(sys.argv) > 1):
    with open(sys.argv[1], 'rb') as f:
        coordCsv = csv.reader(f)
        coordCsv = list(coordCsv)

     #convert string list to float list
    for a in range (0, len(coordCsv)):
        coordCsv[a] = map(float, coordCsv[a])

    for i in range (1, len(coordCsv)):
         #omit 1st and last point from angle calculations
        if i in range(0, len(coordCsv) - 1):
            currVect = (coordCsv[i][0] - coordCsv[i-1][0], coordCsv[i][1] - coordCsv[i-1][1])
            nextVect = (coordCsv[i+1][0] - coordCsv[i][0], coordCsv[i+1][1] - coordCsv[i][1])
            angle = angle_between(currVect, nextVect)


        else:
            angle = 0

        if isDeg:
            angle = constrain(angle, -180, 180)
        else:
            angle = constrain(angle, -pi, pi)

        tupleList.append((coordCsv[i][0], coordCsv[i][1], angle))

    #write data to file
    print tupleList
    f = open(sys.argv[2] + ".yaml", 'w')
    for t in range(0, len(tupleList)):
        n = str(t + 1)
        f.write("/waypoints/" + n + "/x: " + str(tupleList[t][0]) + "\n")
        f.write("/waypoints/" + n + "/y: " + str(tupleList[t][1]) + "\n")
        f.write("/waypoints/" + n + "/theta: " + str(tupleList[t][2]) + "\n")
    f.close()

else:
    print "Enter the filename that contains the coordinate pairs"


