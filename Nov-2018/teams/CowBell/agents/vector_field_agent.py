import argparse
import pickle
import os
import sys

from collections import deque

import numpy as np

import rospy
from std_msgs.msg import Bool, Int16
from geometry_msgs.msg import Point, Twist, PointStamped, Vector3
import host.utilities as util
import host.constants as Constants

def xyToSdgYaw(vx, vy):

    dirRad = 0 #-np.pi/2

    EPSILON = 0.00001

    #print(" (vx_yaw, xy_yaw) = ({}, {}) ".format(vx,vy))

#    if vy > 0.0 and vx > 0.0 and np.abs(vx) > EPSILON:
#        dirRad = np.arctan( vy / vx )
#    elif vy > 0.0 and vx < 0.0 and np.abs(vy) > EPSILON:
#        dirRad = np.pi / 2 + np.arctan( np.abs(vx) / vy )
#    elif vy < 0.0 and vx < 0.0 and np.abs(vx) > EPSILON:
#        dirRad = - np.pi + np.arctan( np.abs(vy) / np.abs(vx) )
#    elif vy < 0.0 and vx > 0.0 and np.abs(vy) > EPSILON:
#        dirRad = - 1/2 * np.pi + np.arctan( np.abs(vx) / np.abs(vy) )

    if vy > EPSILON and vx > EPSILON and np.abs(vx) > EPSILON:
        #print("case 1")
        dirRad += np.arctan( vy / vx )
    elif vy > EPSILON and vx < -EPSILON and np.abs(vy) > EPSILON:
        #print("case 2")
        dirRad += np.pi / 2 + np.arctan( np.abs(vx) / vy )
    elif vy < EPSILON and vx < -EPSILON and np.abs(vx) > EPSILON:
        #print("case 3")
        dirRad = (-np.pi + np.arctan( np.abs(vy) / np.abs(vx) ))
    elif vy < EPSILON and vx > EPSILON and np.abs(vy) > EPSILON:
        #print("case 4")
        dirRad = (-np.pi / 2 + np.arctan( np.abs(vx) / np.abs(vy) ))
    elif vy > EPSILON and np.abs(vx) < EPSILON:
        #print("case 5")
        dirRad = -np.pi/2
    elif np.abs(vy) < EPSILON and vx < -EPSILON:
        #print("case 6")
        dirRad = np.pi / 2
    elif vy < -EPSILON and np.abs(vx) < EPSILON:
        #print("case 7")
        dirRad  = -np.pi
    elif np.abs(vy) < EPSILON and vx > EPSILON:
        #print("case 8")
        dirRad = 0.0

    return dirRad

class VectorFieldAgent():

    VEC_PICKLE_FILE = 'agents/vec.pickle_py2'
    
    if sys.version_info >= (3,0):

        VEC_PICKLE_FILE = 'agents/vec.pickle_py3'

    VEC_PICKLE_KEY = 'vec'

    VEC_PICKLE_OPPOSITE_KEY = 'vecOp'

    # Helper functions
    def set_center(self, sphere_center):

        # old way
        #self.blue_center = sphere_center
        #print("sphere_center = ({}, {})".format(sphere_center.point.x, sphere_center.point.y))

        # new way
        self.blue_center = util.mm_2_pixel(sphere_center.point)
        #print("self.blue_center = ({}, {})".format(self.blue_center.x, self.blue_center.y))
        centern = np.array((0,0))
        x = np.float(sphere_center.point.x)
        y = np.float(sphere_center.point.y)
        #print("x,y = ({},{})".format(x,y))
        centern = np.array((x, y)) / np.float(Constants.ARENA_WIDTH_PIXELS)
        

        print("centern = ({},{})".format(centern[0], centern[1]))
        self.centern = centern

        #print("self.centern = {}\n".format(self.centern))
        return
    
    def calcVel(self, old_pt, new_pt):

       delta_t = ((new_pt.header.stamp.secs
          + new_pt.header.stamp.nsecs / 1000000000.)
          - (old_pt.header.stamp.secs
          + old_pt.header.stamp.nsecs / 1000000000.))

       if np.abs(delta_t) < 0.0001:
          delta_t = 0.1

       vel = np.array((
            (new_pt.point.x
             - old_pt.point.x) / delta_t,
            (new_pt.point.y
             - old_pt.point.y) / delta_t))

       return vel

    def set_flag(self, flag_status):
        self.blue_flag = flag_status.data
        return

    def set_game_over(self, game_state):
        self.game_over = game_state.data
        return

    def set_game_state(self, state):
        if(state.data == 2):
            self.game_over = True
        self.game_state = state.data

    def set_blue_base(self, base):
        self.blue_base = base
        return

    def set_red_base(self, base):
        self.red_base = base
        return

    def yaw_vel_to_twist(self, yaw, vel):
        twist_msg = Twist()
        twist_msg.linear = Vector3(vel, 0, 0)
        twist_msg.angular.x = 0
        twist_msg.angular.y = 0
        yaw += np.pi / 4 
        twist_msg.angular.z = (1080 + np.rad2deg(yaw))%360
        return twist_msg

    # Agent function
    def updateTwist(self):

        # one speed for now
        speed = 25.0

        t = Twist()

        # convert -0.5, 0.5 normalized position to
        # index into image vector
        posArr = self.posIdx(self.centern)
        #print("posArr = ({}, {})".format(posArr[0], posArr[1]))
        #print("centern = ({}, {})".format(self.centern[0], self.centern[1]))

        # default to image that sends us to 
        goal = self.vec
        goalD = self.vecD
        if self.blue_flag:
            #print("our base")
            goal = self.vecOp
            goalD = self.vecOpD
        #vx = goal[int(posArr[0]), int(posArr[1]), 0]
        #vy = goal[int(posArr[0]), int(posArr[1]), 1]
        vx = goal[int(posArr[1]), int(posArr[0]), 1]
        vy = goal[int(posArr[1]), int(posArr[0]), 0]
        #vx = goal[int(posArr[0]), int(posArr[1]), 0]
        #vy = goal[int(posArr[0]), int(posArr[1]), 1]
        #t.angular.x = vx
        #t.angular.y = vy
        #print("(x, y) = ({}, {})".format(int(posArr[0]), int(posArr[1])))
        #print("(gd, gd) = ({}, {})".format(goalD[int(posArr[1]), int(posArr[0]), 1], goalD[int(posArr[1]), int(posArr[0]),
        #    0]))
        #print("(vx, vy) = ({}, {})".format(vx, vy))
        yaw = xyToSdgYaw(vx, vy)

        #print("yaw = {}, vel = {} ". format(yaw * 180.0 / np.pi, speed))

        #print("game_state = {}".format(self.game_state))
        # send twist when game state is test mode or 
        if self.game_state == 1 or self.game_state == 3:
            self.blue_twist = self.yaw_vel_to_twist(yaw, speed)
        return

    def scaleCamVel(self, agentId):

        self.velVec = self.calcVel(self.cam_cen[agent][1], self.cam_cen[agent][0])
        
    def writeCamPos(self, data):
       
        self.cam_data.append(data) 

    def calWithVel(self): 
        
          
        cur_hdg = (np.pi/2) - np.arctan2(self.velVec[0], self.velVec[1])

        if (np.abs(cur_hdg) * 180 / np.pi) < 3.0:
             
           self.pub_set_heading.publish(0)


    def __init__(self, agentName = 'blue_sphero', side="blue"):

        # Global variables
        self.blue_center = Point()
        self.blue_flag = False
        self.blue_base = Point()
        self.red_base = Point()
        self.blue_twist = Twist()
        self.game_over = False
        self.game_state = 0
        self.accumulated_error = 0.
        self.centern = np.array((0, 0))
        self.vec_dtype_max = 256
        self.counter = 0

        self.calHeading = 0
        self.cam_data = deque(maxlen = 3)
        self.velVec = np.array((0,0))

        rospy.init_node(agentName, anonymous=True)
       
        self.pub_blue_cmd = rospy.Publisher('/' + agentName + '/cmd_vel', Twist, queue_size=1)
        self.sub_blue_center = rospy.Subscriber('arena/' + agentName + '/center_mm', PointStamped, self.set_center, queue_size=1)
        self.sub_blue_flag = rospy.Subscriber('/arena/' + agentName + '/flag', Bool, self.set_flag, queue_size=1)
        self.sub_blue_base = rospy.Subscriber('/arena/' + agentName + '/base_mm', Point, self.set_blue_base, queue_size=1)
        self.sub_red_base = rospy.Subscriber('/arena/' + agentName  + '/base_mm', Point, self.set_red_base, queue_size=1)
        self.sub_game_over = rospy.Subscriber('/arena/game_state', Int16, self.set_game_state, queue_size=1)
        self.pub_set_reset_heading = rospy.Publisher(agentName + '/reset_heading', Int16, queue_size=1)
        self.pub_set_heading = rospy.Publisher(agentName + '/set_heading', Int16, queue_size=1)
        self.sub_cam_cen = rospy.Subscriber('/arena/' + agentName + '/center_mm', PointStamped, queue_size=1) 

        print("side = {}".format(side))
        self.side = side
        if side is "blue":
           self.vec = self.loadVecField(VectorFieldAgent.VEC_PICKLE_FILE, VectorFieldAgent.VEC_PICKLE_KEY)
           self.vecOp = self.loadVecField(VectorFieldAgent.VEC_PICKLE_FILE, VectorFieldAgent.VEC_PICKLE_OPPOSITE_KEY)
        else:
           self.vec = self.loadVecField(VectorFieldAgent.VEC_PICKLE_FILE, VectorFieldAgent.VEC_PICKLE_OPPOSITE_KEY)
           self.vecOp = self.loadVecField(VectorFieldAgent.VEC_PICKLE_FILE, VectorFieldAgent.VEC_PICKLE_KEY)

        

        self.rate = rospy.Rate(5)


        # debug
        self.vecD = self.loadVecField(VectorFieldAgent.VEC_PICKLE_FILE, VectorFieldAgent.VEC_PICKLE_OPPOSITE_KEY)

        # debug
        self.vecOpD = self.loadVecField(VectorFieldAgent.VEC_PICKLE_FILE, VectorFieldAgent.VEC_PICKLE_KEY)


    # load the vector field from memory
    # starts out life as an 8bit rgb
    def loadVecField(self, path, key):

        vecDict = pickle.load(open(path, 'rb'))

        vec = vecDict[key]

        vec_norm = vec.astype(np.uint8, copy=False)
        
        vec_norm = (vec_norm - 128.0) / 128.0

        return vec_norm[:, :, 0:2]

    # load the vector field from memory
    # starts out life as an 8bit rgb
    def loadVecFieldDebug(self, path):

        vecDict = pickle.load(open(path, 'rb'))

        return vecDict[key]

    # Get the index into the vector field image, 1,2 np array
    def posIdx(self, vec):

        # -0.5 -> 0.5 in x and y 
        #    to
        # 0 -> 255 in x and y

        # red is y, greed is x, sorry
        vec[1] = -vec[1]
        vec[0] = vec[0]

        vec_idx = ((vec)+ 0.5 ) * ((self.vec_dtype_max) - 1)

        vec_idx = np.around(vec_idx)

        vec_idx = np.minimum(vec_idx, [255, 255])

        vec_idx = np.maximum(vec_idx, [0, 0])

        if self.counter % 10 == 0:
 
           print("vec_idx = ({}, {})".format(vec_idx[0], vec_idx[1]))

        # image indicies are swapped
        #rtnArr = np.array((vec_idx[1], vec_idx[0]))

        # 2nd index is reversed as of the new sphere games
        rtnArr = np.array((vec_idx[0], vec_idx[1]))

        return rtnArr


    # get the twist message, publish, then sleep 
    # to avoid over controlling
    def update(self):

        self.updateTwist()

        #print("blue_twist = {}".format(self.blue_twist)) 
        self.pub_blue_cmd.publish(self.blue_twist)

        self.rate.sleep()

        self.counter += 1
  
        #if self.counter % 2000 == 0 and (self.game_state == 1 or self.game_state == 3):
            #print("reset heading")
            #self.pub_set_reset_heading.publish(0)
        #if self.counter % 20 == 0 and (self.game_state == 1 or self.game_state == 3):
            #print("reset heading")
            #self.pub_set_heading.publish(0)
            #print("cal with vel")
            #self.calWithVel()


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()

        parser.add_argument('-s', '--side', default="blue", help="red or blue")
        parser.add_argument('-i', '--rospy_id', default="blue_sphero", help="rospy id of the agent")

        args = parser.parse_args()

        vfa = VectorFieldAgent(args.rospy_id, args.side)

        #vfa.pub_set_reset_heading.publish(180)
        while not rospy.is_shutdown():

            vfa.update()

            #if vfa.game_over != False:

                #break


    except rospy.ROSInterruptException:
        pass

