import math
import random
import mediapipe as mp
import cv2
import numpy as np
import KALMAN_FILTER as pacheckKalman
import time
import numpy
from Running_Statistics import Running_Stats as RunningStats

# STEP 1: Import the necessary modules.
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision


from cvzone.HandTrackingModule import HandDetector
import cv2






'''
THIS SCRIPT CONTAINS THE REVISED COMPUTER VISION MODULE FOR GORA:

RUNING THIS SCRIPT WILL INICIATE A DEMO FOR THE COMPUTER VISION FOR GORA, 
BUT WILL NOT CONNECT TO GORA ALONE, SEE MAIN.

'''





#SAME AS ARDUINO MAP() FUNCTION
def changeBasis(value, fromLow, fromHigh, toLow, toHigh):
    try:return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow
    except ZeroDivisionError as ZERO:
        print(ZERO)
        return 0


class ComputerVisionBody:
    #This is called the constructor: They give the object different "attributes"
    # "self" is referencing the object, or the "self". More specifically it is in reference to the instance of the object
    #I will talk more about the "instance" of the object later

    '''TLDR: This function gets called when you iniciate the instance of the object'''

    def __init__(self, ACTIVE_KALMAN, cap = cv2.VideoCapture(0)):

        #An attribute that contains a path to methods for drawing on image
        self.mpDraw = mp.solutions.drawing_utils

        # Inicialized the Machine Learning for the pose detection. Is constructor from solutions
        #This attribute contains the solutions from the Machine Learning (ML) Library "Mediapipe"
        self.mpPose = mp.solutions.pose

        # Defines a pose through method beloinging to mpPose constructor under solutions class
        #Contains information about what the results of the ML where
        self.pose = self.mpPose.Pose(static_image_mode=False, min_detection_confidence=0.5, min_tracking_confidence=0.5)

        # Starts up the camera but doesnt take any pictures, just decarles camera.
        # For robotic arm we are going toconfigure the camer here
        self.cap = cap


        #COORDINATE LIST ATTRIBUTES FOR LANDMARKS ON BODY
        self.cx_list = []
        self.cy_list = []
        self.cz_list = []

        #INIT KALMAN CLASS
        self.kalman =  pacheckKalman.KalmanFilter()
        #BOOLEAN FOR RATHER IF KALMAN LAYER EXISTS FOR PREDICITNG ANGLES
        self.active_kalman = ACTIVE_KALMAN

        #RUNNING STATISTICS FOR BASE
        self.Rs4Base = RunningStats()
        self.area = 0


        self.coord_min, self.coord_max = -200, 200

        self.hand_detector = HandDetector(detectionCon=0.8, maxHands=2)

    def landmarks(self, results, img):  # returns a list of points for which cx and cy for certain landmarks

        try:

            #RESET THE COORDINATE LISTS
            self.cx_list, self.cy_list, self.cz_list =  [], [] ,[]


            #IF A PERSON IS IN FRAME
            if results.pose_landmarks:

                #DRAW LANDMARKS ON HUMAN ON IMAGE THAT OUTLINES THE BODY
                self.mpDraw.draw_landmarks(img, results.pose_landmarks, self.mpPose.POSE_CONNECTIONS,
                                           self.mpDraw.DrawingSpec(color=(55,175,212), thickness=3, circle_radius=2),
                                           self.mpDraw.DrawingSpec(color=(1,0,139), thickness=4, circle_radius=2))


                #LOOPS OVER A ENUMERATED SET OF POSE LANDMARKS
                    #THE landmark, lm, has x and y values which define the landmarks ratio wrt the shape of img
                for id, lm in enumerate(results.pose_landmarks.landmark):
                    #GET SHAPE OF IMAGE MATRIX
                    self.h, self.w, self.c = img.shape
                    #GET COORDINATE FOR LANDMARK ID
                    cx, cy, cz = float(lm.x * self.w), float(lm.y * self.h), float(lm.z * self.c)


                    #APPEND THE COORDINATE TO COORDINATE LISTS
                    self.cx_list.append(int(cx))
                    self.cy_list.append(int(cy))
                    self.cz_list.append(int(cz))

            #IF A PERSON IS NOT IN FRAME
            elif results.pose_landmarks == None:
                print("No landmark")
                self.cx_list = []
                self.cy_list = []
                self.cz_list = []


        except AttributeError as AE:
            print("attribute error", AE)
            pass


    #RETURNS IMAGE WITH SKELOTON DRAWN AND MAKES COORDINATE LISTS UPDATED BY ATTRIB REF
    def imageProcessing(self):


        #READ IMAGE
        success, img = self.cap.read()

        # Pose detection algorith works in (red,green,bue) while cv library works in BGR, thus we need to convert.
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  #

        # Now we Process the pose in the new color formatting. Is a class that contains the pose solutions
        results = self.pose.process(imgRGB)

        # Find coordinates of user (X,Y,Z)
            # self.(cx_list, cy_list, cz_list) are now packed for iteration
        self.landmarks(results, img)

        # results.pose_landmarks is a boolean value determineing if a person is in frame or not
        if results.pose_landmarks is not None:
            personBool = True  # Person is in frame
        else:
            personBool = False  # no body in frame

        return img, personBool, success




    def compute_cv_angle(self, p1, p2, p3):
        '''THIS FUNCTION COMPUTES THE ANGLE BETWEEN 3 POINTS P1, P2, AND P3
            WHICH ARE ENUMERATED ACCORDING TO LANDMARK IDs.

            COMPUTES IN 3D SPACE, should be called after image proccessing

            INPUTS:
                p1, p2, p3 where p2 is the rotation point
                self.(cx_list, cy_list, cz_list)

            OUTPUTS:
                Angle '''

        try:

            # Calculate the points defined by the paramters p1, p2, p3
            point1 = (self.cx_list[p1], self.cy_list[p1], self.cz_list[p1])  # make a coordiante (x, y, z) for p1
            point2 = (self.cx_list[p2], self.cy_list[p2], self.cz_list[p2]) # make a coordiante (x, y, z) for p2
            point3 = (self.cx_list[p3], self.cy_list[p3], self.cz_list[p3])  # make a coordiante (x, y, z) for p3


            #GIVEN 3 POINTS IN 3 DIM COORDINATE SPACE, DEFINE 2 VECOTRS
            v1 = np.array([point2[0] - point1[0],
                           point2[1] - point1[1],
                           point2[2] - point1[2]])  # V1 == (X_initcial - X_final, Y_init - Y_final)

            v2 = np.array([point3[0] - point2[0],
                           point3[1] - point2[1],
                           point3[2] - point2[2]])  # V2 == (X_initcial - X_final, Y_init - Y_final)

            #COMPUTE DOT PRODUCT
            v1dotv2 = np.dot(v1, v2)
            #COMPUTE MAGNITUDES OF EACH VECTOR
            v1_mag = np.linalg.norm(v1)
            v2_mag = np.linalg.norm(v2)

            # Calculate the angle using the dot product and magnitudes: solving for theta
            angle = math.acos(v1dotv2 / (v1_mag * v2_mag))

            # Convert angle to degrees
            angle = math.degrees(angle)

            return angle

        except IndexError as IE:
            print("INDEX ERROR IN CALCULATING CV ANGLE", IE)
            return 0

    def clickFlag(self, p1=12, p2=22, conf_x=40, conf_y=40):
        ''' Function determines if 2 points on body are touching or not,
         and returns boolean value for if touch is true or false

         defaults to shoulder touch
            '''
        xList = self.cx_list
        yList = self.cy_list
        try:
            if abs(xList[p2] - xList[p1]) < conf_x and abs(
                    yList[p2] - yList[p1]) < conf_y:  # if click defined by p1 and p2 nearness
                click_flag = True  # Click is true
            else:
                click_flag = False  # Click is false

            return click_flag
        except IndexError as IE:
            print(IE)
            return False



    def find_base_angle(self):
        try:
            #CONVERT TO INTEGER LSIT
            cx_list = [int(value) for value in self.cx_list]

            #RUN STATISTICS
            minValue , maxValue = self.Rs4Base.running_stats(cx_list[16])
            #todo, could also try mapppign from image shape attrib

            thetaB1 = changeBasis(cx_list[16],minValue , maxValue, 0, 360)
            thetaB2 = 0

            thetaBase = thetaB2 + thetaB1

            return int(thetaBase)
        except IndexError as IE:
            pass



    def get_3dof_angles(self):
        try:
            base = 0#int(self.find_base_angle())
            shoulder = int(self.compute_cv_angle(23, 11, 13))
            elbow = int(self.compute_cv_angle(11, 13, 15))
            return base, shoulder, elbow

        except TypeError as TE:
            print(TE)
            pass



    def generate_z_coord(self,point1, point2, minVal, maxVal):

        x1, y1 = point1[0], point1[1]
        x2, y2 = point2[0], point2[1]
        #300, 5000
        area = abs(x2 - x1) * abs(y2 - y1)
        print(area, "area", self.cz_list[16])
        try:
            zbox =int(changeBasis(area + 4*self.cz_list[16], 9200-4*self.cz_list[16], 4*self.cz_list[16]+ 220000, minVal, maxVal)) #int((area - 0) * (maxVal - minVal) / (float('inf') - 0) + minVal)
            return area, zbox
        except ZeroDivisionError as ZERO:
            print(ZERO)
            return area,0

    def get_ik_coords(self,  hands, hand):

        if len(hands) == 2:
            hand = hand
        elif len(hands) == 1:
            hand =0
        print(len(hands), hand, "handn")

        try:
            # Define start and end points of the rectangle
            start_point = (int(min(hands[hand]["lmList"][:][0])), int(min(hands[hand]["lmList"][:][1])))
            end_point = (int(max(hands[hand]["lmList"][:][0])), int(max(hands[hand]["lmList"][:][1])))
            #Generate box around hand (16, 20)
            area, z_box = (self.generate_z_coord(start_point, end_point, self.coord_min, self.coord_max))
            self.area = area
            print("points",tuple(start_point), tuple(end_point), z_box)

            #Get z coordinate from size of bbox
            myik_coords = [changeBasis(self.cx_list[16], 0, 600, -200, 200)
                , changeBasis(self.cy_list[16], 0,500, -200, 200 ),
                         z_box]

            myik_coords = [int(coord) for coord in myik_coords]
        except IndexError as HandMissing:
            print("Hand Missing")
            myik_coords = [200, 0, 200]


        return myik_coords

    def detect_hand_landmarks(self,img):
        # Find the hand and its landmarks
        hands, img = self.hand_detector.findHands(img)  # with draw
        # hands = detector.findHands(img, draw=False)  # without draw



        return img, hands







#This is the function that should be called by other module to get human tracking setup
def inLoop_vision_body(vision):
    try:
        #Get list of body coordinate landmarks, image, and boolean of if person is in frame
        img, personBool, suc = vision.imageProcessing()

        #Generate angles from human left arm angles
        newThetaList = vision.get_3dof_angles()
        #TODO, GET ANGLES FROM BLUETOOTH

        img, hands  = vision.detect_hand_landmarks(img)

        myik_coords = vision.get_ik_coords( hands, 1)

        kalmanData = enumerate(newThetaList)
        if vision.active_kalman:
            newThetaList = [vision.kalman.predict(data) for data in kalmanData]

        #set shoudler bool
        shoulderBool = vision.clickFlag(conf_x=40, conf_y = 40)

        return newThetaList, personBool, img, shoulderBool, suc, myik_coords

    except TypeError as TE:
        print("skipping type error", TE)
        pass




#todo======
# Make 3D computer vision system
if __name__ == "__main__":

    #INIT COMPUTER VISION CLASS
    visionc = ComputerVisionBody(False)


    while True:

        #FOR MEASURING LOOP TIME
        start = time.time()

        #THE IMPORTANT FUNCTION CALLED EACH LOOP, DOES ALL CV STUFF

        newThetaList, personBool, img, shoulderBool, suc= inLoop_vision_body(visionc)

        print(f"thetas{newThetaList}, coords{visionc.cx_list[16], visionc.cy_list[16], visionc.cz_list[16]}")


        #============= INSERT CODE FOR CHANGING BASIS ON MOVMENTS TO MATCH ROBOT



        #=============
        # Show the image
        cv2.imshow("This is the name of the image box", img)
        # Interface timing such that it looks like a video
        cv2.waitKey(1)


        end = time.time()
        loopTime = abs(start - end)

