

############# Imports ####################

import sys
#sys.path.append(r'c:\users\giuseppe\appdata\local\packages\pythonsoftwarefoundation.python.3.10_qbz5n2kfra8p0\localcache\local-packages\python310\site-packages')
from djitellopy import Tello
import cv2 
import numpy as np
from pupil_apriltags import Detector


################### Initialization ##############


w,h = 360,240  # frame size


# Tune here #   

pid = [0.35,0.6,0]  # PID parameter for yaw control
pid_d = [12,10,0]    # PID parameter for forward motion control

### Tuning end ###

pError = 0 # previous error in yaw initialized

pError_dis = 0  # previous error in distance initialized
startCounter = 0  # flag to take off drone , set to 1 if don't want to fly the drone

# camera parameter
intrinsic_matrix = np.array([[921.170702, 0.000000, 459.904354], [0.000000, 919.018377, 351.238301], [0.000000, 0.000000, 1.000000]])
distortion_coeffs  = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000])

# april tag library
at_detector = Detector(
   families="tag36h11",
   nthreads=1,
   quad_decimate=1.0,
   quad_sigma=0.0,
   refine_edges=1,
   decode_sharpening=0.25,
   debug=0
)

tag_size = 0.063 
object_points = np.array([
            [-0, -0, 0],
            [tag_size, 0, 0],
            [tag_size, tag_size, 0],
            [0, tag_size, 0]
        ], dtype=np.float32)




##########################  Functions #################################


def intializeTello():
   # CONNECT TO TELLO
   myDrone = Tello()
   myDrone.connect()
   myDrone.for_back_velocity = 0
   myDrone.left_right_velocity = 0
   myDrone.up_down_velocity = 0
   myDrone.yaw_velocity = 0
   myDrone.speed =0
   myDrone.streamoff()
   myDrone.streamon()
   return myDrone



def telloGetFrame(myDrone,w=360,h=240):
    # GET THE IMGAE FROM TELLO
    myFrame = myDrone.get_frame_read()
    myFrame = myFrame.frame
    img = cv2.resize(myFrame, (w, h))
    return img

def findTag(image):

    img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    faces = at_detector.detect(img,estimate_tag_pose= False)
    myFacesListC = []
    myFaceListArea = []
    translation_vector = [0]

    for detection in faces:
        image_points = detection.corners.astype(float)
        (ptA, ptB, ptC, ptD) = detection.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))

        cv2.line(image, ptA, ptB, (0, 0, 255), 2)
        cv2.line(image, ptB, ptC, (0, 0, 255), 2)
        cv2.line(image, ptC, ptD, (0, 0, 255), 2)
        cv2.line(image, ptD, ptA, (0, 0, 255), 2)
  
        x ,y = int(ptA[0]), int(ptA[1])
        w = abs(int(ptB[0])- int(ptA[0]))
        h = abs(int(ptA[1])- int(ptD[1]))
        cx = detection.center[0]
        cy = detection.center[1]
        area = w*h
        success, rotation_vector, translation_vector = cv2.solvePnP(object_points, image_points, intrinsic_matrix, distortion_coeffs)
        
        myFacesListC.append([cx,cy])
        myFaceListArea.append(area)
 
    if len(myFaceListArea) != 0:
         i = myFaceListArea.index(max(myFaceListArea))
         # index of closest face
         return image,[myFacesListC[i],myFaceListArea[i]], translation_vector[-1]     
    else:
         return image, [[0,0],0], translation_vector[-1]


def trackFace(myDrone,c,w,dis,pid,pError,pError_dis):
     
     
     error_dis = dis - 0.5   # error in forward distance   # 1 m is minimum distance b/w drone and tag
     error = c[0][0] - w//2   # error in yaw
     

     speed = int(pid[0]*error + pid[1] * (error-pError))
     speed = int(np.clip(speed, -100, 100))

     f_speed = int(pid_d[0]*error_dis + pid_d[1] * (error_dis-pError_dis))
     f_speed = int(np.clip(f_speed, -100, 100))
     

     if dis > 0.5:
        myDrone.for_back_velocity = f_speed
     
     else:
        myDrone.left_right_velocity = 0
        myDrone.for_back_velocity = 0
        myDrone.up_down_velocity = 0
        myDrone.yaw_velocity = 0
        error_dis = 0
        

     if c[0][0] != 0:
        myDrone.yaw_velocity = speed
     else:
        myDrone.left_right_velocity = 0
        myDrone.for_back_velocity = 0
        myDrone.up_down_velocity = 0
        myDrone.yaw_velocity = 0
        error = 0


      # SEND VELOCITY VALUES TO TELLO
     if myDrone.send_rc_control:
        myDrone.send_rc_control(myDrone.left_right_velocity,myDrone.for_back_velocity,myDrone.up_down_velocity, myDrone.yaw_velocity)

     return error,error_dis



######################## Main ###################################


myDrone = intializeTello()

while True:



   if startCounter == 0 :
      myDrone.takeoff()
      #myDrone.move_up(20)
      

      startCounter = 1

   ## STEP 1
   img = telloGetFrame(myDrone)
   # DISPLAY IMAGE

   ## STEP 2
   img, info,dis = findTag(img)
   
   cv2.imshow("MyResult", cv2.resize(img, (1080, 720)))

   ## STEP 3
   pError,pError_dis = trackFace(myDrone,info,w,dis,pid,pError,pError_dis)

   # WAIT FOR THE 'Q' BUTTON TO STOP
   if (cv2.waitKey(1) & 0xFF )== ord('q'):
      print('a')
   # replace the 'and' with '&amp;' 
      myDrone.land()
      break

