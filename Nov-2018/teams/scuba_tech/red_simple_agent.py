import time
import numpy as np
import rospy
from std_msgs.msg import Bool, Int16
from geometry_msgs.msg import Point, Twist, Vector3

# Global variables
blue_center = Point()
blue_flag = False
red_center = Point()
red_flag = False
blue_base = Point()
red_base = Point()
blue_twist = Twist()
game_over = False
game_state = 0
accumulated_error = 0.
neutral_zone = False
section_zone = 1
zone = 0

# Helper functions
def set_center(sphere_center):
    global blue_center
    blue_center = sphere_center
    return


def set_red_center(sphere_center):
    global red_center
    red_center = sphere_center
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

def set_game_state(state):
    global game_over, game_state
    if(state.data == 2):
        game_over = True
        game_state = state.data
    else:
        game_state = state.data

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
    if(vel>250): vel = 250
    twist_msg.linear = Vector3(vel, 0, 0) 
#    twist_msg.angular.x = np.cos(yaw) * vel 
#    twist_msg.angular.y = np.sin(yaw) * vel 
    twist_msg.angular.x = 0 
    twist_msg.angular.y = 0 
    twist_msg.angular.z = yaw
    return twist_msg

def get_heading_and_distance():
    global blue_center, blue_flag, blue_base, red_base, neutral_zone, section_zone, zone

    #Zones
    # 2   3
    # 0   1
    zone = 0
    if(blue_center.x >= (0.5 * (max(blue_base.x, red_base.x) - min(blue_base.x, red_base.x)) + min(blue_base.x, red_base.x))):
        zone = zone + 1   
    if(blue_center.y <= (0.5 * (max(blue_base.y, red_base.y) - min(blue_base.y, red_base.y)) + min(blue_base.y, red_base.y))):
        zone = zone + 2



    if  blue_flag:
        if(zone == 3):
            target_x = (0.5 * (max(blue_base.x, red_base.x) - min(blue_base.x, red_base.x)) + min(blue_base.x, red_base.x))
            target_y = (0.25 * (max(blue_base.y, red_base.y) - min(blue_base.y, red_base.y)) + min(blue_base.y, red_base.y))
        if(zone == 2):
            target_x = (0.25 * (max(blue_base.x, red_base.x) - min(blue_base.x, red_base.x)) + min(blue_base.x, red_base.x))
            target_y = (0.5 * (max(blue_base.y, red_base.y) - min(blue_base.y, red_base.y)) + min(blue_base.y, red_base.y))
        if(zone == 1):
            target_x = (0.5 * (max(blue_base.x, red_base.x) - min(blue_base.x, red_base.x)) + min(blue_base.x, red_base.x))
            target_y = (0.75 * (max(blue_base.y, red_base.y) - min(blue_base.y, red_base.y)) + min(blue_base.y, red_base.y))
        if(zone == 0):
            target_x = blue_base.x
            target_y = blue_base.y
        distance = np.sqrt((blue_center.x - blue_base.x) ** 2 + (blue_center.y - blue_base.y) ** 2)

    else:
        if(zone == 0):
            target_x = (0.25 * (max(blue_base.x, red_base.x) - min(blue_base.x, red_base.x)) + min(blue_base.x, red_base.x))
            target_y = (0.5 * (max(blue_base.y, red_base.y) - min(blue_base.y, red_base.y)) + min(blue_base.y, red_base.y))
        if(zone == 1):
            target_x = (0.75 * (max(blue_base.x, red_base.x) - min(blue_base.x, red_base.x)) + min(blue_base.x, red_base.x))
            target_y = (0.5 * (max(blue_base.y, red_base.y) - min(blue_base.y, red_base.y)) + min(blue_base.y, red_base.y))
        if(zone == 2):
            target_x = (0.5 * (max(blue_base.x, red_base.x) - min(blue_base.x, red_base.x)) + min(blue_base.x, red_base.x))
            target_y = (0.25 * (max(blue_base.y, red_base.y) - min(blue_base.y, red_base.y)) + min(blue_base.y, red_base.y))
        if(zone == 3):
            target_x = red_base.x
            target_y = red_base.y
        distance = np.sqrt((blue_center.x - red_base.x) ** 2 + (blue_center.y - red_base.y) ** 2)


    # if neutral_zone and blue_flag:
    #     # Have flag, go home
    #     target_x = blue_base.x
    #     target_y = blue_base.y
    # elif not blue_flag and (neutral_zone != False):
    #     # Don't have flag, go to opponent's base
    #     target_x = red_base.x
    #     target_y = red_base.y
    # else:
    #     # Haven't passed through neutral zone, go there
    #     target_x = (0.25 * (max(blue_base.x, red_base.x) 
    #                       - min(blue_base.x, red_base.x)) 
    #                       + min(blue_base.x, red_base.x))
    #     target_y = (0.25 * (max(blue_base.y, red_base.y) 
    #                       - min(blue_base.y, red_base.y)) 
    #                       + min(blue_base.y, red_base.y))

    # if blue_flag:
    #     if(section_zone == 3):
    #         target_x = (0.5 * (max(blue_base.x, red_base.x) - min(blue_base.x, red_base.x)) + min(blue_base.x, red_base.x))
    #         target_y = (0.25 * (max(blue_base.y, red_base.y) - min(blue_base.y, red_base.y)) + min(blue_base.y, red_base.y))
    #     if(section_zone == 2):
    #         target_x = (0.25 * (max(blue_base.x, red_base.x) - min(blue_base.x, red_base.x)) + min(blue_base.x, red_base.x))
    #         target_y = (0.5 * (max(blue_base.y, red_base.y) - min(blue_base.y, red_base.y)) + min(blue_base.y, red_base.y))
    #     if(section_zone == 1):
    #         target_x = blue_base.x
    #         target_y = blue_base.y
    # else:
    #     if(section_zone == 1):
    #         target_x = (0.25 * (max(blue_base.x, red_base.x) - min(blue_base.x, red_base.x)) + min(blue_base.x, red_base.x))
    #         target_y = (0.5 * (max(blue_base.y, red_base.y) - min(blue_base.y, red_base.y)) + min(blue_base.y, red_base.y))
    #     if(section_zone == 2):
    #         target_x = (0.5 * (max(blue_base.x, red_base.x) - min(blue_base.x, red_base.x)) + min(blue_base.x, red_base.x))
    #         target_y = (0.25 * (max(blue_base.y, red_base.y) - min(blue_base.y, red_base.y)) + min(blue_base.y, red_base.y))
    #     if(section_zone == 3):
    #         target_x = red_base.x
    #         target_y = red_base.y

    # target_x = red_center.x
    # target_y = red_center.y
                     
    delta_x = target_x - blue_center.x
    delta_y = target_y - blue_center.y
    distance = np.sqrt(delta_x ** 2 + delta_y ** 2)
    print("[{}, {}, {}, {},{},{},{},{}]".format(delta_x, delta_y,distance, zone,target_x,target_y,blue_center.x,blue_center.y))


    # if (distance < 50):
    #     if(section_zone == 2 and not blue_flag): 
    #         section_zone = 3
    #     if(section_zone == 1 and not blue_flag): 
    #         section_zone = 2
    #     if(section_zone == 2 and (blue_flag)): 
    #         section_zone = 1
    #     if(section_zone == 3 and (blue_flag)): 
    #         section_zone = 2
        
    if not neutral_zone and distance < 50:
        neutral_zone = True
    heading = np.arctan2(-delta_x, delta_y)
    heading = heading * 180 / 3.1415
    heading += 180
    if (heading < 0): 
	heading += 360 
    elif (heading > 360):
        heading -= 360 
    return heading, distance

# Agent function
def proportional_control():
    global blue_twist, accumulated_error, game_state, zone, blue_flag
    
    print('Entering proportional control')
    print('This is blue center: ', blue_center)
    if blue_center != Point():
        heading, distance = get_heading_and_distance()
	print('This is heading: ', heading)
	print('This is distance: ', distance)
        #heading = -heading # Switch from camera to world coordinates
        if distance < 100:
            accumulated_error = 0
        else:
            accumulated_error += distance
        speed = distance / 10. + accumulated_error / 10000.
    else:
        speed = 0
        heading = 0
    if(speed<10):speed=10    
    blue_twist = yaw_vel_to_twist(heading, speed)
    print('This is blue twist', blue_twist)
    return

# Init function
def simple_agent():
    global game_over, game_state
    
    print('Entering ROS simple agent')
    # Setup ROS message handling
    rospy.init_node('red_agent', anonymous=True)

    print('Ros initiated')
    pub_blue_cmd = rospy.Publisher('/red_sphero/cmd_vel', Twist, queue_size=1)
    sub_blue_center = rospy.Subscriber('/arena/red_sphero/center', Point, set_center, queue_size=1)
    sub_red_center = rospy.Subscriber('/arena/blue_sphero/center', Point, set_red_center, queue_size=1)
    sub_blue_flag = rospy.Subscriber('/arena/red_sphero/flag', Bool, set_flag, queue_size=1)
    sub_blue_base = rospy.Subscriber('/arena/red_sphero/base', Point, set_blue_base, queue_size=1)
    sub_red_base = rospy.Subscriber('/arena/blue_sphero/base', Point, set_red_base, queue_size=1)
    #sub_game_over = rospy.Subscriber('/game_over', Bool, set_game_over, queue_size=1)
    sub_game_state = rospy.Subscriber('/arena/game_state', Int16, set_game_state, queue_size=1)

    print('Subscribed and publishing')
    # Agent control loop
    rate = rospy.Rate(2) # Hz
    while not rospy.is_shutdown():
	print('Waiting for Start')
	while(game_state == 0):time.sleep(0.001)
	print("Started")
        if(0):
            pub_blue_cmd.publish(yaw_vel_to_twist(0, 0))
        else:
            print('Entering else statement')
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
