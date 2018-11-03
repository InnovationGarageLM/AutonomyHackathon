import numpy as np
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Twist, Vector3
from random import randint
from kalmanFilter2 import kalmanFilter

# Global variables
blue_center = Point()
blue_flag = False
blue_base = Point()
red_base = Point()
blue_twist = Twist()
game_over = False
accumulated_error = 0.
neutral_zone = False

# Helper functions
def set_center(sphere_center):
    global blue_center
    blue_center = sphere_center
    return

def set_flag(flag_status):
    global blue_flag, neutral_zone
    # Logic for needing to go back through neutral zone
    if blue_flag != flag_status.data:
        neutral_zone = False
    blue_flag = flag_status.data
    return

def set_game_over(game_state):
    global game_over
    game_over = game_state.data
    return

def set_blue_base(base):
    global blue_base
    blue_base = base
    return

def set_red_base(base):
    global red_base
    red_base = base
    return

def yaw_vel_to_twist(yaw, vel): 
    twist_msg = Twist() 
    twist_msg.linear = Vector3(0, 0, 0) 
    twist_msg.angular.x = np.cos(yaw) * vel 
    twist_msg.angular.y = np.sin(yaw) * vel 
    twist_msg.angular.z = 0 
    print( twist_msg.angular.x, twist_msg.angular.y)
    return twist_msg

def get_heading_and_distance():
    global blue_center, blue_flag, blue_base, red_base, neutral_zone
    if neutral_zone and blue_flag:
        # Have flag, go home
        target_x = blue_base.x
        target_y = blue_base.y
    elif not blue_flag and (neutral_zone != False):
        # Don't have flag, go to opponent's base
        target_x = red_base.x
        target_y = red_base.y
    else:
        # Haven't passed through neutral zone, go there
        target_x = (0.25 * (max(blue_base.x, red_base.x) 
                          - min(blue_base.x, red_base.x)) 
                          + min(blue_base.x, red_base.x))
        target_y = (0.25 * (max(blue_base.y, red_base.y) 
                          - min(blue_base.y, red_base.y)) 
                          + min(blue_base.y, red_base.y))
    delta_x = target_x - blue_center.x
    delta_y = target_y - blue_center.y
    print("[{}, {}]".format(delta_x, delta_y))
    distance = np.sqrt(delta_x ** 2 + delta_y ** 2)
    if not neutral_zone and distance < 50:
        neutral_zone = True
    heading = np.arctan2(delta_y, delta_x)
    return heading, distance

#Getting grid position
def get_grid_position(X_k):
    global blue_center, blue_flag, red_base, blue_base
    if blue_flag != False:
        target_x = blue_base.x
        target_y = blue_base.y
        flag_ind = 0
    else:
        target_x = red_base.x
        target_y = red_base.y
        flag_ind = 1
    # What x grid?
    print("Target:" , target_x, target_y)
    maxy = max(blue_base.y, red_base.y)
    miny = min(blue_base.y, red_base.y)
    maxx = max(blue_base.x, red_base.x)
    minx = min(blue_base.x, red_base.x)
    n_ygrid = 8
    n_xgrid = 8
    if (maxx == 0) or (minx == 0):
        x_grid = 0
        y_grid = 0
    else:
        #x_grid = int(n_xgrid * (blue_center.y - minx) / (maxx - minx))  + 1 
        x_grid = int(n_xgrid * (X_k[0][0] - minx) / (maxx - minx))  + 1 
        #print(x_grid, blue_center.y, minx, maxx)
        if x_grid > 8:
            x_grid = 8
        
        #x_grid += ((n_xgrid +1) * flag_ind ) # This gives a 16 x 8 grid (8x8 for each flag state)
        #print(x_grid, flag_ind)
        #y_grid = int(8 * (blue_center.x - miny) / (maxy - miny))  + 1  
        y_grid = int(8 * (X_k[2][0] - miny) / (maxy - miny))  + 1  
        if y_grid > 8:
            y_grid = 8
        y_grid += ((n_ygrid + 1) * flag_ind ) # gives a 16 x 8 grid
	#print(blue_center.y, minx, maxx)
        
    #print("xgrid, ygrid: ",x_grid, y_grid)
    #print("X_k[0], X_k[2]: ", X_k[0][0], X_k[2][0])
    #print("x_center, y_center: ", blue_center.x, blue_center.y)
    return x_grid, y_grid, flag_ind


#Creating a Q_weight lookup table
def set_Q_weight():
    global Q_weight
    Q_weight = np.array([
        [50, 50, 50, 50, 50, 50, 50, 50, 50, 50],
        [50,  9,  8,  9, 50, 50, 12, 13, 14, 50], 
        [50,  8,  7,  8,  9, 11, 11, 12, 13, 50],
        [50,  7,  6,  7,  8,  9, 11, 11, 12, 50],
        [50, 50,  5,  6, 50, 50,  9, 10, 50, 50],
        [50, 50,  4,  5, 50, 50,  8,  9, 50, 50],
        [50,  2,  3,  4,  5,  5,  7,  8,  9, 50],
        [50,  1,  2,  3,  4,  5,  6,  7,  8, 50],
        [50,  0,  1,  2, 50, 50,  7,  8,  9, 50],
        [50, 50, 50, 50, 50, 50, 50, 50, 50, 50],
        [50,  9,  8,  7, 50, 50,  2,  1,  0, 50],
        [50,  8,  7,  6,  5,  4,  3,  2,  1, 50],
        [50,  9,  8,  7,  6,  5,  4,  3,  2, 50],
        [50, 50,  9,  8, 50, 50,  5,  4, 50, 50],
        [50, 50, 10,  9, 50, 50,  6,  5, 50, 50],
        [50, 12, 11, 10,  9,  8,  7,  6,  7, 50],
        [50, 13, 12, 11, 10,  9,  8,  7,  8, 50],
        [50, 14, 13, 12, 50, 50,  9,  8,  9, 50],
        [50, 50, 50, 50, 50, 50, 50, 50, 50, 50]])



# best choice:
def best_choice(x_grid,y_grid, flag_ind):
    global Q_weight, blue_center, blue_flag
    if (blue_flag   ==1) and (x_grid == 17) and (y_grid == 1):
        return -1.67
    if (blue_flag   ==0) and (x_grid == 1) and (y_grid == 8):
        return 1.67
    # look at x_grid+/-1, y_grid+/-1
    center_value = Q_weight[y_grid][x_grid]
    min_value = center_value
    dirx = 0
    diry = 0
    for i in range(3):
        tempx = i-1
        for j in range(3):
            tempy = j-1
            checkval = Q_weight[y_grid + tempy][x_grid + tempx]
            if checkval < min_value:
                min_value = checkval
                dirx = tempx
                diry = tempy
    
    if (dirx == 0) and (diry == 0):
        dirx = randint(0,1) - 0.5
        diry = randint(0,1) - 0.5
    #Now I have the i,j. Translate that to direction
    #direc = np.array([[135, 180, 215],[90, 0, 270], [45, 0, 315]])
    #bdir = direc[i][j] * 3.14159 /180 #convert to radians. Not sure if this is right.
    bdir = np.arctan2(-1 * diry,dirx)
    print(x_grid,y_grid, dirx, diry, bdir)
    return bdir

#Calculating Z_k
def Zk_calc():
    global previous_center, blue_center  #, X_k1, rp_rate
    Zk0 = blue_center.x
    Zk1 = (blue_center.x - previous_center.x) * rp_rate
    Zk2 = blue_center.y
    Zk3 = (blue_center.y - previous_center.y) * rp_rate
    Z_k = np.array([[Zk0], [Zk1], [Zk2], [Zk3]])
    return Z_k

# Agent function
def proportional_control():
    global Q_weight, blue_twist, accumulated_error, previous_angle, previous_center, blue_center, X_k1, P_k1, R_k, rp_rate
    
    speed = 9
    px = previous_center.x
    py = previous_center.y
    cx = blue_center.x
    cy = blue_center.y
    measured_angle = np.arctan2((cy-py),(cx-px))  #
    print("Measured angle, previous angle:", measured_angle, previous_angle)
    difference = measured_angle - previous_angle
    
    Z_k = Zk_calc()
    if X_k1[0][0] == 0:
        X_k1 = np.array([[blue_center.x], [0], [blue_center.y], [0]])  # X and y are backwards.  My fault.


    deltaT = 1/rp_rate #2 Hz?
    X_k, P_k = kalmanFilter(X_k1, P_k1, Z_k, R_k, deltaT, previous_angle, speed)


    # get grid position

    x_grid, y_grid, flag_ind = get_grid_position(X_k)
    direction = best_choice(x_grid, y_grid, flag_ind)
    
    KF_heading = np.arctan2(X_k[3][0], X_k[1][0])  # y / x
    head_error = previous_angle - KF_heading
    #head_error = 0
    heading = direction + 0.2* head_error + 0.2* accumulated_error
    
    #heading = -1 * heading
    #blue_twist = yaw_vel_to_twist(heading, speed)
    blue_twist = yaw_vel_to_twist(direction, speed)
    alpha = .2 #slow down error accumulation
    accumulated_error = (1 - alpha) * accumulated_error + alpha * difference
    #print("Total Error:", accumulated_error)
    #previous_angle = heading
    previous_angle = heading
    previous_center = blue_center
    X_k1 = X_k
    P_k1 = P_k
    return

# Init function
def simple_agent():
    global game_over, blue_center,previous_center, accumulated_error, previous_angle, X_k1, P_k1, R_k, rp_rate

    # Setup ROS message handling
    rospy.init_node('blue_agent', anonymous=True)

    pub_blue_cmd = rospy.Publisher('/blue_sphero/twist_cmd', Twist, queue_size=1)
    sub_blue_center = rospy.Subscriber('/blue_sphero/center', Point, set_center, queue_size=1)
    sub_blue_flag = rospy.Subscriber('/blue_sphero/flag', Bool, set_flag, queue_size=1)
    sub_blue_base = rospy.Subscriber('/blue_sphero/base', Point, set_blue_base, queue_size=1)
    sub_red_base = rospy.Subscriber('/red_sphero/base', Point, set_red_base, queue_size=1)
    sub_game_over = rospy.Subscriber('/game_over', Bool, set_game_over, queue_size=1)
    
    #Set values for X_k1, P_k1
    X_k1 = np.array([[blue_center.x], [0], [blue_center.y], [0]])  # X and y are backwards.  My fault.
    P_k1 = np.identity(4) * 0;
    R_k = np.array([[0.1, 0, 0, 0],[0, 0.8, 0, 0],[0, 0, 0.1, 0],[0, 0, 0, 0.8]])
    
    previous_center = blue_center
    previous_angle = 0
    accumulated_error = 0.0
    set_Q_weight()
    # Agent control loop
    rp_rate = 3  #2 Hz.
    rate = rospy.Rate(rp_rate) # Hz
    while not rospy.is_shutdown():
        proportional_control()
        pub_blue_cmd.publish(blue_twist)
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

