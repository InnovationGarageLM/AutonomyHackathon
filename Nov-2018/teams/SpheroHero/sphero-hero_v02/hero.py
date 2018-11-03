#! /usr/bin/env python

'''
Sphero Hero agent

Three command line parameters: {red|blue} [odom|camera] [test]
  Required - the sphero color to control
  Optional - use sphero odometer instead of camera (less accurate)
  Optional - enable test output and visual image

Odometer mode is not supported in the simulator due to lack of ability
to reset positioning.

Topics subscribed:
  /arena/game_state [Int16]: 0-wait, 1-in play, 2-done
  /arena/{color}_sphero/flag [Bool]: True/false if have flag
  /arena/{color}_sphero/center [Point]: location of sphero based on camera
  /{color}_sphero/odometry [PointStamped]: sphero location per onboard telemetry
  /{color}_sphero/velocity [PointStamped]: sphero velocity per onboard telemetry

Topics read once on init:
  /arena/red_sphero/base [Point]: location of red sphero's base
  /arena/blue_sphero/base [Point]: location of blue sphero's base

Topics published:
  /{color}_sphero/cmd_vel [Twist]: command sphero movement
  /{color}_sphero/reset_heading [Int16]: reset sphero heading to zero
  /{color}_sphero/set_position [Point]: reset sphero telemetry
[Latter two not used in simulator]

When in test mode:
  Subscribe {camera image topic} [CompressedImage]: camera image
  Publish /{color}_sphero/hero_test [Image]: test image showing move decision

'''
##############################################################################

from __future__ import print_function
import traceback, random
import time, sys, os

import rospy
import numpy as np
from std_msgs.msg import Bool, Int16
from geometry_msgs.msg import Point, PointStamped, Twist, Vector3
from sensor_msgs.msg import Image, CompressedImage

import cv2
from cv_bridge import CvBridge

from arena import Arena, APoint, ABounds

# Flag if in simulator or physical, default physical
SIMULATOR = os.getenv('AUTONOMY_SIMULATOR', False)

# Flag to suppress debug and image generation/display
TESTING = False

# Topic for camera image
if SIMULATOR:
  CAMERA_TOPIC = '/camera/rgb/image_raw/compressed'  #gazebo
else:
  CAMERA_TOPIC = '/raspicam_node/image/compressed'   #physical

# Use sphero odometry or camera tracker?
USE_ODOM = False #optional set from command line

# Measured pixel/sphero unit computation
SCALAR = ( 0, 0 ) #computed on start

# Set from command line
WE_ARE, THEY_ARE = None,None

# game state enums
WAITING, PLAYING, ENDGAME = 0,1,2

class agent(object):
    def __init__(self, arena):
        self.arena = arena
        self.bridge = self.rawimg = None
        self.last_tick = time.time()-100 #something in the past
        self.last_trk = arena.center
        self.last_odom = APoint(0,0)
        self.opponent = None

        self.game_state = -1
        self.have_flag = None

        self.where = self.last_trk # Where are we now
        self.goto = None           # Where are we trying to go
        self.last_angle = 0.0      # Should be facing forward
        self.reset_heading = 0.0   # Last heading when reset

        self.good_trk_count = 0
        self.zero_vel_count = 0
        self.unstick_count = 10

        self.wait_trk = self.good_trk = False
        self.not_moving = False

        #self.points = np.zeros((7,2)) #lines for tracking

        # Conversion pixels/sphero units - consider using center instead of base
        self.toPixels = lambda pt: (pt * SCALAR) + self.arena.mybase
        self.toSphero = lambda pt: (pt - self.arena.mybase) / SCALAR

    def ros_setup(self):
        sphero = '/'+WE_ARE+'_sphero'

        self.pub_move = rospy.Publisher(
            sphero+'/cmd_vel', Twist, queue_size=1)
        if not SIMULATOR:
            self.pub_rset_head = rospy.Publisher(
                sphero+'/reset_heading', Int16, queue_size=1)
            self.pub_rset_pos = rospy.Publisher(
                sphero+'/set_position', Point, queue_size=1)

        rospy.Subscriber('/arena/game_state', Int16,
                         self.told_game, queue_size=1)
        rospy.Subscriber('/arena'+sphero+'/flag', Bool,
                         self.told_flag, queue_size=1)
        rospy.Subscriber('/arena'+sphero+'/center', Point,
                         self.told_tracker, queue_size=1)
        rospy.Subscriber('/arena/'+THEY_ARE+'_sphero/center', Point,
                         self.told_opponent, queue_size=1)
        rospy.Subscriber(sphero+'/odometry', PointStamped,
                         self.told_odometry, queue_size=1)
        rospy.Subscriber(sphero+'/velocity', PointStamped,
                         self.told_velocity, queue_size=1)

        if TESTING: 
            self.bridge = CvBridge()
            self.pub_show = rospy.Publisher(
                sphero+'/hero_test', Image, queue_size=1) 
            rospy.Subscriber(CAMERA_TOPIC, CompressedImage,
                             self.told_image, queue_size=1)
    #end

    def told_image(self,img): #callback(CompressedImage)
        '''Given camera raw image (used in testing only)'''
        self.rawimg = img

    def told_game(self,flg): #callback(Int16)
        '''Given game state: 0=wait, 1=play, 2=done 3=test'''
        if flg.data != self.game_state:
            how = { WAITING:'waiting start', PLAYING:'playing',
                    ENDGAME:'game over', 3:'test'}
            say = how[flg.data]
            if say is None: say=('unknown (%d)'%flg.data)
            print("GAME STATE: now",say)
            self.game_state=flg.data

    def told_flag(self,flg): #callback(Bool)
        '''Given current game flag; true we have it, false we don't'''
        if flg.data != self.have_flag:
            if TESTING: print('FLAG CHANGE: we do%s have flag'
                              % ('' if flg.data else ' not'))
            self.have_flag = flg.data

    def told_opponent(self, loc):  #callback(Point)
        '''Given opponent location based on camera tracker; just save the
        location for use by movement code'''
        #if self.game_state != PLAYING: return  #ignore if not in game
        self.opponent = APoint(loc)
        # We are NOT waiting for a tracker settle time here, only care about
        # rough position; leave opponent=None to ignore opponent avoidance

    def told_tracker(self, loc):  #callback(Point)
        '''Given sphero location based on camera tracker; requires
        a settle time of returning same location +/- 10 pixels 3 times 
        in a row'''
        if self.game_state != PLAYING: return  #ignore if not in game

        loc = APoint(loc)
        if loc.close_to(self.last_trk,10): # +/-10
            self.good_trk_count += 1
            if self.good_trk_count >= 3:
                if TESTING and not self.good_trk:
                    print("Stable tracker %g,%g after %d count" 
                          % (loc.x,loc.y,self.good_trk_count))
                self.good_trk = True
        else:
            self.last_trk = loc
            self.good_trk_count = 1
            self.good_trk = False

    def told_odometry(self, loc): #callback(PointStamped)
        '''Given sphero location from internal telemetry, just save the
        location for use in movement.'''
        #PointStamped(header(seq,stamp(sec,nsec)),point)
        #if self.game_state != PLAYING: return  #ignore if not in game
        self.last_odom = self.toPixels(APoint(loc.point))

    def told_velocity(self, loc): #callback(PointStamped)
        '''Given sphero velocity from internal telemetry, if zero for
        three times in a row then we consider ourselves not moving'''
        #PointStamped(header(seq,stamp(sec,nsec)),point)
        if self.game_state != PLAYING: return  #ignore if not in game

        zero = APoint(loc.point).close_to(APoint(0),20)  #+/-20
        if zero:
            self.zero_vel_count += 1
            if self.zero_vel_count >= 3:
                if TESTING and not self.not_moving:
                    print("STUCK at zero velocity for %d count"
                          % (self.zero_vel_count))
                self.not_moving = True
        else:
           self.zero_vel_count = 0
           self.not_moving = False

    #------------------------------------------------------------------------
    def tick(self):
        '''Execute an action for this time tick'''
        now = time.time()
        if self.game_state != PLAYING:
            if self.last_tick+10 < now:
                self.last_tick=now
                print("Waiting for the game to start")
            return
        #else game in play!

        if self.wait_trk or self.not_moving:
            if self.good_trk:
                self.reset_sphero(self.last_trk)
                self.wait_trk = False

            if self.not_moving:
                use = random.choice([ 90, 135, 180, 225, 270 ])
                if TESTING: print("TWIST to %g get unstuck"
                                  % (self.last_angle-use))
                self.send_twist(25,self.last_angle-use)
                self.not_moving = False
                self.unstick_count = 0
                self.zero_vel_count=0

        self.do_movement()
        if TESTING: self.showit()
        self.last_tick=now

    def do_movement(self):
        '''Make the movement for the current tick'''
        loc = self.last_odom if USE_ODOM else self.last_trk
        self.where = loc
        #self.points = np.append(self.points[1:], [(loc.x,loc.y)], 0)

        #Note current logic will not get here with not_moving=True

        if self.wait_trk or self.unstick_count <= 5:
            self.unstick_count += 1
        else:
            #Where should we move?
            tgt = self.arena.get_target(loc,self.have_flag,self.opponent)
            if tgt is None:
                if TESTING: print("Out of bounds, wait tracker")
                self.wait_trk = True # Out of bounds, need position reset
            else:
                self.goto = tgt
                vel,angle = self.move_metric(loc,tgt)
                self.send_twist(vel,angle)

    def compute_line(self,img=None):
        '''Compute best-fit line and error of that line.  Unfortunately
        it doesn't return the error (although must compute it internally)
        so I'm computing a sum of distance from each point to the line
        as my error.  There may be a better way to do this but since I
        only have a few points I think it will be fine.

        The idea was that if we moved far enough in a straight line, we could
        reset the sphero orientation.  Unfortunately toward the end of the
        game when it is needed the most, it is harder to get a good straight
        line movement.  Alternate idea is to make a specific straight-line
        move and capture that direction.  But it is too close to game time, it
        doesn't appear we really need this to do well, and it was never fully
        tested/integrated.  Leaving the code here for reference (it shows a
        good picture if nothing else.)'''

        #To use this you need to restore the lines above for 'self.points'

        #Distance from first to last point
        pt0,pt1 = self.points[-1], self.points[0]
        ptdist = np.sqrt((pt1[0]-pt0[0])**2 + (pt1[1]-pt0[1])**2)
        if ptdist < self.arena.bounds.width/8.0: #at least 1/2 quad
            return  #not far enough

        #(pts, type, 0=optimal, 0.01=accuracy radius, 0.01=accuracy angle)
        #return [vect.x, vect.y, pt.x, pt.y] for vector and point on line
        # (x,y) = (pt.x,pt.y) + t *(vec.x,vec.y)
        line = cv2.fitLine(self.points, cv2.DIST_L2, 0, 0.01, 0.01)
        vec = APoint(line[0,0],line[1,0])
        pt = APoint(line[2,0],line[3,0])

        #Distance from point to line
        #  d = abs(-vec.y * x + vec.x * y + (vec.y*pt.x - vec.x*pt.y))
        #       / sqrt(-vec.y**2+vec.x**2)
        c = (vec.y*pt.x - vec.x*pt.y)
        denom = np.sqrt(((-vec.y)**2 + (vec.x)**2))
        dist = lambda pt: abs(-vec.y*pt[0] + vec.x*pt[1] + c) / denom

        if img is not None:     # show computed line
            cv2.line(img, (pt+vec*100).pixels(), (pt-vec*100).pixels(), 
                     (100,100,255), 1)
        sum = 0
        for p in self.points:
            sum += dist(p)
            if img is not None: # show points
                cv2.circle(img,(int(p[0]),int(p[1])),3,(255,255,255),-1)
        sum /= len(self.points)

        if sum < 0.6:
            # This angle computation is WRONG
            angle = np.degrees(-vec.y/vec.x) - 90
            while angle >= 360.0: angle-=360.0
            while angle < 0.0: angle+=360.0

            print("Direction reset at dist %g, error %g, angle %g vs last %g"\
                  % (ptdist,sum,angle,self.last_angle))
            #self.reset_sphero(self.where, angle)
    #end

    def reset_sphero(self, loc=None, heading=None):
        if SIMULATOR: return  #not supported in simulator

        '''Reset sphero location/heading to initial or given'''
        if loc is None and heading is None:
            self.reset_heading = 0.0
            self.pub_rset_head.publish(Int16(0))
            self.pub_rset_pos.publish(Point(0,0,0))
            if TESTING: print("Sphero reset initial heading and position")

        if loc is not None:
            loc = self.toSphero(loc)  #convert from pixel to sphero
            self.pub_rset_pos.publish(Point(loc.x,loc.y,0))
            if TESTING: print("Sphero position set to %g,%g"%(loc.x,loc.y))
        if heading is not None:
            #To allow resetting heading, must track angular offset
            self.reset_heading = heading
            self.last_angle = 0.0 #last angle is now current heading
            self.pub_rset_head.publish(Int16(0)) # (value unused here)
            if TESTING: print("Sphero heading reset to %g"%(heading))

    def move_metric(self, src, tgt):
        '''Return vel,angle for the move'''
        dist = np.sqrt((tgt.x-src.x)**2 + (tgt.y-src.y)**2)
        vel = (2*dist / self.arena.bounds.width) * 40.0
        vel = max(20, min(40, vel))  #clip 20..40

        x,y = (tgt - src).tuple() #Sphero wants degrees, clockwise, zero north
        angle = np.degrees(np.arctan2(y,x)) + 90
        angle -= self.reset_heading #TODO: test this!
        while not (0.0 <= angle < 360.0): #shouldn't need loop
            if angle < 0.0: angle += 360.0
            if angle >= 360.0: angle -= 360.0
        #if TESTING: print("Moving %s to %s: vel %g, angle %g"
        #                  % (src,tgt,vel,angle))
        return vel,angle

    def send_twist(self, vel,angle):
        '''Create/publish the twist message for given vel/angle'''
        while not (0.0 <= angle < 360.0): #shouldn't need loop
            if angle < 0.0: angle += 360.0
            if angle >= 360.0: angle -= 360.0

        twist = Twist() 
        twist.linear = Vector3(0,0,0) 
        twist.angular = Vector3(0,0,0)
        twist.linear.x = vel    #velocity
        twist.angular.z = angle #direction (deg)
        self.last_angle = angle # Save what we last did
        self.pub_move.publish(twist)

    #------------------------------------------------------------------------
    # Only called when testing
    def showit(self):
        '''Create/publish an image showing our planned movement'''
        if self.rawimg is not None:
            white,green = (255,255,255),(50,150,50)
            yellow,red = (0,200,200),(0,0,255)
            img = self.bridge.compressed_imgmsg_to_cv2(self.rawimg)

            loc = self.where
            cv2.circle(img,loc.pixels(),15,green,2)
            if USE_ODOM: cv2.circle(img,self.last_trk.pixels(),15,red,2)

            # Restore this call to get the line-fitting debug
            #self.compute_line(img)

            tgt = self.goto
            if tgt is not None:
                cell = self.arena.what_cell(loc)
                if cell is not None:
                    ctr,hlf = self.arena.cell_center(cell),self.arena.half_cell
                    cv2.rectangle(img,(ctr-hlf).pixels(),(ctr+hlf).pixels(),
                                  yellow,2)
                vel,ang = self.move_metric(loc,tgt)
                cv2.putText(img,("%d,%d"%(int(vel),int(ang))), loc.pixels(), 
                            cv2.FONT_HERSHEY_SIMPLEX,1,white,2)
                cv2.arrowedLine(img,loc.pixels(),tgt.pixels(),green,2)
            img = self.bridge.cv2_to_imgmsg(img)
            self.pub_show.publish(img)

#end
##############################################################################

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: agent {red|blue} [odom|camera] [test]")
        sys.exit(0)

    if sys.argv[1] == 'red':
        WE_ARE, THEY_ARE ='red','blue'
    elif sys.argv[1] == 'blue':
        WE_ARE, THEY_ARE = 'blue','red'
    else:
        print("Unrecognized color '%s', try red or blue" % sys.argv[1])
        sys.exit(0)

    if len(sys.argv)>2 and sys.argv[2] == 'odom': 
        USE_ODOM=True
        if SIMULATOR:
            print("The simulator version cannot use the odometer as that"\
                  " won't allow resetting the positioning.")
            sys.exit(0)

    if len(sys.argv)>3 and sys.argv[3] == 'test': TESTING=True

    print("We are '%s', they are '%s'" % (WE_ARE,THEY_ARE))
    print("Using %s for location tracking" %
          ('odometry' if USE_ODOM else 'camera'))

    try:
        # Setup ROS message handling
        rospy.init_node('agent', anonymous=True)

        # We only need base locations once so we'll just read one message
        # instead of repeatedly.
        print("Waiting for the two base locations")
        our_base = rospy.wait_for_message(
            '/arena/'+WE_ARE+'_sphero/base', Point)
        their_base = rospy.wait_for_message(
            '/arena/'+THEY_ARE+'_sphero/base', Point)
        print("  Bases: ours at (%d,%d), theirs at (%d,%d)" 
              % (our_base.x,our_base.y,their_base.x,their_base.y))

        # Need to initialize our arena object
        print("Bases received, initializing arena")
        arena = Arena()
        arena.set_bases(our_base, their_base)

        # Compute pixel/sphero scalar - using 650 sphero steps from base to
        # origin (only used if tracking by sphero telem)

        SCALAR = ( arena.center - arena.mybase ) / 650.0
        if arena.myquad == 0: SCALAR *= (+1,-1)
        if arena.myquad == 1: SCALAR *= -1  #negate
        if arena.myquad == 2: SCALAR *= +1
        if arena.myquad == 3: SCALAR *= (-1,+1)
        if TESTING: print("Pixel/sphero scalar: %g,%g" % (SCALAR.x,SCALAR.y))

        # Now startup the agent
        ag = agent(arena)
        ag.ros_setup()
        ag.reset_sphero()

        print("Arena initialized, agent ready")
        rate = rospy.Rate(12) # Hz
        while not rospy.is_shutdown() and ag.game_state != ENDGAME:
            ag.tick()
            rate.sleep()
        #end
        print("Game has apparently ended, terminating")

    except rospy.ROSInterruptException as e:
        pass
    except Exception as e:
        print(e); traceback.print_exc()
#end
