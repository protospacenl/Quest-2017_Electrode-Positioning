# import the necessary packages
from collections import deque
import serial
import time
import numpy as np
import argparse
import imutils
import cv2
import sys
import struct

PY3 = sys.version_info[0] == 3
if PY3:
    xrange = range

LB = 120
UB = 45
RB = 480
DB = 405

LS = 200
US = 125
RS = 400
DS = 300

FORWARD = 80 
BACK = 120


OLD_CASE = 0



#serial connection Arduino
ser = serial.Serial('/dev/ttyS0', 9600)

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (110, 50, 50)
greenUpper = (130, 255, 255)
pts = deque(maxlen=args["buffer"])

#webcam
camera = cv2.VideoCapture(1)


# keep looping
while True:
	# grab the current frame
	(grabbed, frame) = camera.read()

	# if we are viewing a video and we did not grab a frame,
	# then we have reached the end of the video
	if args.get("video") and not grabbed:
		break

	# resize the frame, blur it, and convert it to the HSV
	# color space
	frame = imutils.resize(frame, width=600)
	#blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	

	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
	center = None
	

	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))


		# only proceed if the radius meets a minimum size
		if radius > 25:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius),
                                (0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)

##				and (center[0]-radius < LS and center[1]-radius < US and center[0]+radius > RS and center[1]+radius > DS)		    


			if (radius < FORWARD) and OLD_CASE != 5:
				ser.write(b'f')	
				print ("Forward")
				OLD_CASE = 5

			elif (radius > BACK) and OLD_CASE != 6:
				ser.write(b'b')	
				print ("Back")							    
				OLD_CASE = 6
				
			elif (center[0] > LS  and center[1] > US and center[0] < RS and center[1] < DS) and (OLD_CASE != 7) and (radius > FORWARD) and (radius < BACK):
				ser.write(b's')	 
				print ("stop")
				OLD_CASE = 7
			elif center[0] > RS and OLD_CASE != 1:
				ser.write(b'r')	
				print ("right")
				OLD_CASE = 1

			elif center[0] < LS and OLD_CASE != 2:
				ser.write(b'l')	
				print ("left")
				OLD_CASE = 2

			elif center[1] < US and OLD_CASE != 3:
				ser.write(b'u')	
				print ("Up")
				OLD_CASE = 3

			elif center[1] > DS and OLD_CASE != 4:
				ser.write(b'd')	
				print ("Down")
				OLD_CASE = 4

			#print(radius)


	

	# update the points queue
	pts.appendleft(center)
	
	if center == None:
		time.sleep(.05)
		if center == None:
			ser.write(b's')	
			print ("Stop None")


	cv2.rectangle(frame,(LB,UB),(RB,DB),(0,255,0),2) #Big rect
	cv2.rectangle(frame,(LS,US),(RS,DS),(0,255,0),2) #Small rect


	# show the frame to our screen
	cv2.imshow("hsv", hsv)
	cv2.imshow("mask", mask)
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF

	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break
        

# cleanup the camera and close any open windows
ser.write(b's')	
camera.release()
cv2.destroyAllWindows()
