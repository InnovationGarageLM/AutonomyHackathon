import numpy as np
import rospy
from std_msgs.msg import Bool, Int16
from geometry_msgs.msg import Point, Twist, Vector3
from simple_pid import PID
#from scipy.spatial import distance as scipy_distance
import collections
import time 

##### Tuning values #####
my_color = "blue" #Set to red or blue
Hz = 7.5 #rospy.rate in Hz. More hurts is more better. Assuming it doesn't break everything (default 2)

#########################
# Global variables
pid = PID(-.06,.00001,.0008, setpoint=0)
pid.sample_time = .25
ind_lead = 250

paths = []
my_base_set = False
op_base_set = False
path_set = False
path_max_ind = 0
path_ind = 0
on_path = False
my_score = 0
op_score = 0 

my_center = Point()
op_center = Point()

my_buffer = []
my_buffer_length = 30
op_buffer = []
op_buffer_length = 20
yaw_bias = 0

my_flag = False
op_flag = False
my_flag_changed = False
op_flag_changed = False
flag_counter = 0
my_base = Point()
op_base = Point()
my_twist = Twist()
game_over = False
game_state = 0
accumulated_error = 0.
neutral_zone = False

sub_my_base = None
sub_op_base = None

corners = [] #variable to hold list of corners of each quadrant to get unstuck
is_stuck = False
unsticking = False
distance = 0

counter = 0

op_consecutive_points = 0

##### Globals from Sphere team six #####
neutralA = Point()                  #points roughly in the center of each neutral zone
neutralB = Point()                  #these two are used as references to determine our preferred path based on opponent proximity
prefPath = "B"                      #indicates the preferred waypoint path to take A is upper, B is lower

## SUBSCRIPTION CALLBACK FUNCTIONS ##
def set_my_center(sphere_center):
    global my_center, my_buffer, my_buffer_length
    my_center = sphere_center
    my_buffer.append(my_center)
    if len(my_buffer) > my_buffer_length:
        my_buffer.pop(0)
    return

def set_op_center(sphere_center):
    global op_center, op_buffer, op_buffer_length
    op_center = sphere_center
    op_buffer.append(op_center)
    if len(op_buffer) > op_buffer_length:
        op_buffer.pop(0)
    return

def set_flag(flag_status):
    global my_flag
    my_flag = flag_status.data
    return

def set_op_flag(flag_status):
    global op_flag
    op_flag = flag_status.data
    return

def set_my_score(score):
    global my_score
    my_score = score.data
    return

def set_op_score(score):
    global op_score
    op_score = score.data
    return

def set_game_over(game_state_msg):
    global game_over
    game_over = game_state_msg.data
    return

def set_game_state(game_state_msg):
    global game_state
    game_state = game_state_msg.data
    return

def set_my_base(base):
    global my_base, my_base_set, sub_my_base
    #print("MY BASE GETTING SET")
    my_base = base
    #print("MY BASE: ",my_base.x,my_base.y)
    my_base_set = True
    #print("OP_BASE SET?",op_base_set)
    if op_base_set:
        set_path()
    sub_my_base.unregister()
    return

def set_op_base(base):
    global op_base, op_base_set, sub_op_base
    #print("OP BASE GETTING SET")
    op_base = base
    #print("OP BASE: ",op_base.x,op_base.y)
    op_base_set = True
    #print("MY_BASE SET?",my_base_set)
    if my_base_set:
        set_path()
    sub_op_base.unregister()
    return
## END SUBSCRIPTION CALLBACK FUNCTIONS ##

def find_corners():
    #Calculates corner points based on red/blue base locations
    global my_base, op_base
    corners.append((my_base.x,my_base.y))
    corners.append((my_base.x,op_base.y))
    corners.append((op_base.x,op_base.y))
    corners.append((op_base.x,my_base.y))
    return

def path_pref(center):                   
    #determine opponent location and set preferred path
    #To use, check prefPath global for the preferred path
    global pathX, prefPath

    #get oppenent's location
    oCenter = center

    #Calculate oppenents distance to path A
    prox2A = np.sqrt((neutralA.x - oCenter.x)**2 + (neutralA.y - oCenter.y)**2)
    #Calculate oppenents distance to path B
    prox2B = np.sqrt((neutralB.x - oCenter.x)**2 + (neutralB.y - oCenter.y)**2)

    #Set prefered path to which ever one is further from oppenent
    if prox2A >= prox2B:
        prefPath = "B"
    if prox2A < prox2B:
        prefPath = "A"

def set_path():
    #After setting both the red and blue base locations,
    #calculate the path and save in global variable 'path'
    global op_base, my_base, op_base_set, my_base_set, paths, path_set, path_max_ind
    if my_base_set and op_base_set and (not path_set):#Make sure base locations are set
        print("PATH GETTING SET")
        print("OP BASE: ",op_base.x,op_base.y)
        print("MY BASE: ",my_base.x,my_base.y)
        find_corners()#Populate the list of corners

        midx = (op_base.x+my_base.x)/2#Get the midpoint in x dimension between bases
        midy = (op_base.y+my_base.y)/2#Get the midpoint in y dimension between bases
        base_dist_x = np.abs(op_base.x-my_base.x)#Get distance between bases in x dimension
        base_dist_y = np.abs(op_base.y-my_base.y)#Get distance between bases in y dimension

        ##Set path points##
        #Note: Path points were set manually to get the desired shape
        #Set point 1 along path
        p1x = midx - base_dist_x/7
        p1y = midy + base_dist_y/2.75 + 50

        #Set point 2 along path
        p2x = midx + base_dist_x/2.75 + 50
        p2y = midy - base_dist_y/6

        #Handles what color our team is and makes it so that the path always goes from 
        #blue base to red base (increasing x value)
        if my_color == 'blue':
            red_base = op_base
            blue_base = my_base
        else:
            red_base = my_base
            blue_base = op_base

        if blue_base.x > red_base.x:
            tmp_base = blue_base
            blue_base = red_base
            red_base = tmp_base
            reverse_color = 'blue'

        #Calculate the slope of the line that connects the two bases (line 1)
        m1 = (my_base.y-op_base.y)/(my_base.x-op_base.x)
        #Calculate the intercept of the line that connects the two bases (line 1)
        b1 = my_base.y - m1*my_base.x      

        #Calualte slope of line that is perpendicular to the first line (line 2)
        m2 = -m1
        #Calualte intercept of line that is perpendicular to the first line (line 2)
        b2 = m1*midx + b1 - m2*midx

        #Creates lislts of path points for x and y dimension
        path_points_x = [blue_base.x - 100, p1x, p2x, red_base.x]
        path_points_y = [blue_base.y, p1y, p2y, red_base.y]
        
        #Fit a polynomial to the path points
        p = np.polyfit(path_points_x, path_points_y,2)
        
        #Find where the polynomial meets line 2 (defined my m2,b2)
        #Setup quadratic formula
        a = p[0]
        b = (p[1] - m2)
        c = p[2] - b2

        #Quadratic formula yields 2 solutions
        sol1 = (-b + np.sqrt(b**2 - 4*a*c))/(2*a)
        sol2 = (-b - np.sqrt(b**2 - 4*a*c))/(2*a)

        #Correct solution is positive
        if sol1 > 0:
            xint = sol1
        else:
            xint = sol2
	print("BLUEBASEX,XINT",blue_base.x,xint)
        #Create the actual path using an evelny spaced x vector petween the red base and line 2
        #This creates the first half of the path
        pathx = range(int(blue_base.x),int(xint) + 1)
        print("PATHX",pathx)
        #Evaluate polynomial at each x point to create y points
        pathy = np.polyval(p,pathx)
        print("PATHY",pathy)
        
        #Mirror the first half of the path over line 2 to create the second half of the path
        path_mirror = [mirror_point((x,y),m=m2,b=b2) for x,y in zip(pathx,pathy)]
        #reverse the order of points so that this vector can be appended to 1st half path vector
        path_mirror.reverse()

        #Package path as a list of tuples
        path = [(px,py) for px,py in zip(pathx,pathy)]
        path.extend(path_mirror)
        
        #Polynomial isn't perfect so manually set the first point to be the blue base
        path[0] = (blue_base.x, blue_base.y)
        #Polynomial isn't perfect so manually set the last point to be the red base
        path[-1] = (red_base.x, red_base.y)

        #Add this path to the 'paths' list since this is 1 of two possible paths
        paths.append(path)
    
        #Create a mirror of the path over the line which connects the two bases
        path_mirror = [mirror_point(p,m=m1,b=b1) for p in paths[0]]
        #Add the mirrored path to the list of paths
        paths.append(path_mirror)

        #Make it so that index 0 is my base and the last index is oppenent base
        if my_color == reverse_color:
            paths[0].reverse()
            paths[1].reverse()

        #Set max index for the path (python is 0 based)
        path_max_ind = len(path) - 1
        path_set = True#Make it known that the path is set
    return

def mirror_point(p,m=1,b=0):
    #Takes a point, a slope, and an intercept and mirrors the point about the line
    #defined by the slope and intercept

    #Get x and y values from point
    x1 = p[0]
    y1 = p[1]

    #Mirror point
    d = (x1 + (y1 - b)*m)/(1+m**2)
    x2 = 2*d - x1
    y2 = 2*d*m - y1 + 2*b

    #Return the mirrored point
    return((x2,y2))

# def set_yaw_bias():
#     #Calculates the current yaw bias
#     #NOTE: NOT TESTED
#     global yaw_bias, my_buffer, my_buffer_length
#     if len(my_buffer) < my_buffer_length:
#         return
#     else:
#         x = [p.x for p in my_buffer]
#         y = [p.y for p in my_buffer]
#         p = np.polyfit(x,y,2)
#         x0 = x[-5]
#         m = 2*p[0]*x0 + p[1]
#         true_yaw = -np.arctan2(m,1)
#         return

def yaw_vel_to_twist(yaw, vel): 
    #Takes yaw and velocity and conversts it to a 'twist message'
    #before it gets sent to the sphero as a command
    #Note: This function was provided by the competition
    twist_msg = Twist() 
    #twist_msg.linear = Vector3(0, 0, 0) 
    #twist_msg.linear.x = np.cos(yaw) * vel 
    #twist_msg.angular.z = np.sin(yaw) * vel 
    twist_msg.linear.x = vel
    twist_msg.angular.z = yaw
    #twist_msg.angular.z = 0 
    return twist_msg

def check_if_stuck():
    #Checks if our sphero hasn't moved in a while or if it is finished getting unstuck
    global is_stuck, my_buffer, pid, unsticking
    stuck_buffer_length = 20#Number of points to use when determining if stuck
    my_buffer_length = len(my_buffer)#Length of position point buffer (containing previous N positions)
    if my_buffer_length > stuck_buffer_length + 1:#If position point buffer is longer than stuck buffer length...
        if not is_stuck:#If I'm not stuck...
            dists = []#Init distances list
            #Calculate the distance between every point in my buffer and its list neighbors
            for p1,p2 in zip(my_buffer[my_buffer_length-1:my_buffer_length-(stuck_buffer_length+1):-1],my_buffer[my_buffer_length-2:my_buffer_length-(stuck_buffer_length+2):-1]):
                dists.append(dist((p1.x,p1.y),(p2.x,p2.y)))
            if np.max(dists) < 10:#If the maximum distance is less than 50 pixels
                is_stuck = True#Declare that I am stuck
        elif not unsticking:#If I'm stuck, AND I'm not unsticking....
            #Get the distances to each corner on the map
            dist_to_corner = np.min([dist((my_center.x,my_center.y),corner) for corner in corners])
            if dist_to_corner < 100:#If I'm within 100 pixels of the corner...
                unsticking = True#Declare that I am in the process of getting unstuck
        else:#If I'm stuck, and I'm getting unstuck...
            if distance < 75:#If I'm within 50 pixels of my set point...
                #I'm done getting unstuck, set flags appropriately
                is_stuck = False
                unsticking = False
    return

def at_corners():
    #Checks if my sphero is at a base
    global my_center
    #Checks if sphero is at home base
    if (my_center.x in range(int(my_base.x -50), int(my_base.x +50)) and  my_center.y in range(int(my_base.y -50), int(my_base.y +50))):
        print "at my_base corner"
        return True
    #Checks if sphero is at opponent base
    elif (my_center.x in range(int(op_base.x -50), int(op_base.x +50)) and  my_center.y in range(int(op_base.y -50), int(op_base.y +50))):
        return True
    #Returns false if not at a base
    else:
        return False


def get_to_path():
    #Calculates distance and heading to desired setpoint
    #Desired setpoint is set based on strategy, flag status, my location, and opponent location
    global my_flag, paths, my_center, ind_lead, path_max_ind, is_stuck, path_ind, distance, counter
 
    print("IS_STUCK",is_stuck)
    print("UNSTICKING",unsticking)
    if at_corners():
        set_path_ind()

    path = paths[path_ind]
    closest_ind = closest_index(my_center, path)
    if is_stuck:
        if unsticking:
            target = path[closest_ind]
        else:
            target = corners[closest_index(my_center,corners)]
    else:
        if not my_flag:
            target = path[np.min([path_max_ind,closest_ind+ind_lead])]
        else:
            target = path[np.max([0,closest_ind-ind_lead])]
  
     
    target_x = target[0]
    target_y = target[1]
    delta_x = target_x - my_center.x
    delta_y = target_y - my_center.y
    distance = np.sqrt(delta_x ** 2 + delta_y ** 2)
    heading = calc_heading(delta_y, delta_x)
    print("TAGETx: ", target_x)
    print("TAGETy: ", target_y)
    print("DISTANCE: ", distance)
    print("HEADING: ", heading)
 
    return heading, distance

def closest_index(current, full_path):
    #closest_index = distance.cdist(current, full_path).argmin()
    dists = np.array([dist((current.x,current.y),p) for p in full_path])
    closest_ind = dists.argmin()
    return closest_ind

def set_path_ind():
    global op_center, paths, on_path, path_ind, my_center
    op_cen = [op_center.x, op_center.y]
    #op_dist_path_zero = scipy_distance.cdist(op_cen, paths[0]).argmin()
    #op_dist_path_one = scipy_distance.cdist(op_cen, paths[1]).argmin() 
    op_closest_ind_zero = np.array([dist((op_center.x, op_center.y),p) for p in paths[0]]).argmin()
    op_closest_ind_one = np.array([dist((op_center.x, op_center.y),p) for p in paths[1]]).argmin()
    op_distance_zero = np.sqrt((paths[0][op_closest_ind_zero][0] - op_cen[0])**2 + (paths[0][op_closest_ind_zero][1] - op_cen[1])**2)
    op_distance_one = np.sqrt((paths[1][op_closest_ind_one][0] - op_cen[0])**2 + (paths[1][op_closest_ind_one][1] - op_cen[1])**2)
    if my_score + 3 < op_score:
        if op_distance_zero < op_distance_one:#take the same path at opponent
	    path_ind = 0
	    on_path = True
	else:
	    path_ind = 1
	    on_path = True
    else:
        if op_distance_zero < op_distance_one:#avoid opponent
            path_ind = 1
            on_path = True
        else:
            path_ind = 0
            on_path = True

def defense():
    global my_center, op_center, my_base, op_base, paths, ind_lead
    x_dist = op_base.x - my_base.x
    y_dist = my_base.y - op_base.y
 
    print "playing defense"
    home_base_top = (my_base.x, my_base.y - y_dist/2)
    home_base_bot = (my_base.x + x_dist/2, my_base.y)
    #course_center = (x_dist/2, y_dist/2)
    if point_dist(op_center,my_base) < 400:
        op_mid_base = Point((3*my_base.x+op_center.x)/4, (3*my_base.y+op_center.y)/4, 0)
        target_x = op_mid_base.x
        target_y = op_mid_base.y
    else:
        path = paths[0]
        closest_ind = closest_index(my_center, path) 
        target = path[np.max([0,closest_ind-ind_lead])]
        target_x = target[0]
        target_y = target[1]
    delta_x = target_x - my_center.x
    delta_y = target_y - my_center.y
    distance = np.sqrt(delta_x ** 2 + delta_y ** 2)
    heading = calc_heading(delta_y, delta_x)
    return heading, distance

def calc_heading(dy,dx):
    print("DY,DX",dy,dx)
    th = np.arctan2(dy,dx)
    print("TH",th)
    if th < 0:
        th += 2*np.pi
    print("TH",th)
    th += np.pi/2
    print("TH",th)
    new_th = th%(2*np.pi)
    print("TH",new_th)
    th_deg = np.rad2deg(new_th)
    print("TH",th_deg)
    return(th_deg)

def dist(p1,p2):
    x1 = float(p1[0])
    y1 = float(p1[1])
    x2 = float(p2[0])
    y2 = float(p2[1])
    dist = np.sqrt((x2-x1)**2 + (y2-y1)**2)
    return(dist)

def point_dist(p1,p2):
    return(dist((p1.x,p1.y),(p2.x,p2.y)))
 
# Agent function
def proportional_control():
    global my_twist, accumulated_error, pid, unsticking
    if my_center != Point():
        #set_yaw_bias()
        check_if_stuck()
        #print "SCORE my,op: ", my_score, op_score
        if my_score > op_score + 5:
           heading, distance = defense()
        else:
            heading, distance = get_to_path()
       # heading = -heading # Switch from camera to world coordinates
       # heading = np.rad2deg((heading+np.pi))
        speed = pid(distance)
        print("HEADING,SPEED",heading,speed)
    else:
        print "IN ELSE"
        speed = 0
        heading = 0
    my_twist = yaw_vel_to_twist(heading, speed)
    return

# Init function
def simple_agent():
    global game_over, sub_my_base, sub_op_base
    if my_color == 'red':
        op_color = 'blue'
    else:
        op_color = 'red'

    # Setup ROS message handling
    rospy.init_node(my_color + '_agent', anonymous=True) #don't worry about this, it's just the name we have in ROS. Both players can be init as blue afaik -cp

    #To swap teams, set variable 'my_color' at the top of the file
    pub_my_cmd =    rospy.Publisher('/'+my_color+'_sphero/cmd_vel', Twist, queue_size=1) #Added variables -cp
    sub_my_center = rospy.Subscriber('/arena/'+my_color+'_sphero/center', Point, set_my_center, queue_size=1)
    sub_op_center = rospy.Subscriber('/arena/'+op_color+'_sphero/center', Point, set_op_center, queue_size=1) 
    sub_my_flag =   rospy.Subscriber('/arena/'+my_color+'_sphero/flag', Bool, set_flag, queue_size=1)
    sub_op_flag =   rospy.Subscriber('/arena/'+op_color+'_sphero/flag', Bool, set_op_flag, queue_size=1)
    sub_my_score =  rospy.Subscriber('/arena/'+my_color+'_sphero/score', Int16, set_my_score, queue_size=1)
    sub_op_score =  rospy.Subscriber('/arena/'+op_color+'_sphero/score', Int16, set_op_score, queue_size=1)
    sub_my_base =   rospy.Subscriber('/arena/'+my_color+'_sphero/base', Point, set_my_base, queue_size=1)
    sub_op_base =   rospy.Subscriber('/arena/'+op_color+'_sphero/base', Point, set_op_base, queue_size=1)
    sub_game_over = rospy.Subscriber('/arena/game_over', Bool, set_game_over, queue_size=1)
    sub_game_state = rospy.Subscriber('/arena/game_state', Int16, set_game_state, queue_size=1)

    # Agent control loop
    rate = rospy.Rate(Hz) # Hz, tunable. higher = better, upper limit is likely 15
    while not path_set:
        rate.sleep()

    while game_state == 0:
        rate.sleep()

    while not rospy.is_shutdown():
        proportional_control()
        pub_my_cmd.publish(my_twist)
        if game_over != False:
            break
        rate.sleep()
    print("Game ended. No agent to save.")
    return

if __name__ == '__main__':
    try:
        simple_agent()
    except rospy.ROSInterruptException:
        pass
