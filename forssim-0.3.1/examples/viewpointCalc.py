#!/usr/bin/python
import math
import sys

# coordinate system starts in inner lower left corner of kobra table.

# measurements
midX = 0.285 # measured as center of kobra table
screenHeight = 0.3
screenWidth = 0.53
screenTopLeft = {"x": 0.02, "y": 0.207, "z": 0.055}


# y = kz + m (the mirrored screen plane)
k = -1 # screen is angled at 45 deg
m = screenTopLeft["y"] + screenTopLeft["z"]

# screen is in 45 deg, making it's y-diff and z-diff screenHeight / sqrt(2)
zDiff = yDiff = screenHeight / math.sqrt(2)

# x is constant, y and z we get by moving zyDiff / 2 in corresponding directions
screenMidPoint = {
	"x": midX, 
	"y": screenTopLeft['y'] - yDiff / 2, 
	"z": screenTopLeft['z'] + zDiff / 2
	}

print 'Screen mid point: "{0}" "{1}" "{2}"'.format(screenMidPoint['x'], screenMidPoint['y'], screenMidPoint['z'])
userInput = sys.argv[1] or raw_input("Distance from screen: ")

distanceFromScreen = float(userInput)
distanceYZ = distanceFromScreen / math.sqrt(2)

viewpoint = {
	"x": midX, 
	"y": screenMidPoint['y'] + distanceYZ, 
	"z": screenMidPoint["z"] + distanceYZ 
	}

# tan v = opposite / adjacent = (screenHeight / 2) / distanceFromScreen
# We should use the minimum field of view which is given by screen height (the other fov is calculated by h3d from screen ratio)
# see http://www.h3dapi.org/uploads/api/H3DAPI_20/docs/H3DAPI/html/classH3D_1_1Viewpoint.html#_details
fieldOfView = 2 * math.atan((screenHeight / 2) / distanceFromScreen)
print 'Viewpoint position="{0} {1} {2}" fieldOfView="{3}"'.format(viewpoint["x"], viewpoint["y"], viewpoint["z"], fieldOfView)
