#!/usr/bin/env python

from multiprocessing.dummy import current_process
from turtle import position
import rospy
import math
from geometry_msgs.msg import Twist, Point, Polygon
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16


# Constant integers for directions
RIGHT = 1
LEFT = -1
ZERO = 0

Zone_1 = (1, 2, 3, 4)
Zone_2 = (5, 6, 7, 8)

class LineFollowing():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('agv_0/cmd_vel', Twist, queue_size=1)
        self.line_following()
        
    def get_odom(self):
        current_odom = rospy.wait_for_message('agv_0/odom', Odometry)
        return current_odom

    def get_park_command(self):
        park_command = 0
        park_command = rospy.wait_for_message('agv_0/rest', Int16)
        return park_command
    
    def check_marker(self):
        has_marker = False
        marker_id = -1
        # TODO: marker detector
        return has_marker, marker_id
    
    def in_circle(center_x, center_y, radius, x, y):
        square_dist = (center_x - x) ** 2 + (center_y - y) ** 2
        return square_dist <= radius ** 2

  
    def check_side_of_line(A, B, P):
        global RIGHT, LEFT, ZERO
        
        # Subtracting co-ordinates of
        # point A from B and P, to
        # make A as origin
        B.x -= A.x
        B.y -= A.y
        P.x -= A.x
        P.y -= A.y
    
        # Determining cross Product
        cross_product = B.x * P.y - B.y * P.x
    
        # Return RIGHT if cross product is positive
        if (cross_product > 0):
            return RIGHT
            
        # Return LEFT if cross product is negative
        if (cross_product < 0):
            return LEFT
    
        # Return ZERO if cross product is zero
        return ZERO

    def check_side(self, odom):
        side = 0
        # 1. get geometry from odom

        # 2. calculate side
        return side
    
    def check_speed_zone(self, odom):
        speed = 1.5
        # if in speed zone 1
        if(self.inside_rectangle(0, 1, 2, 3, odom.pose.pose.position.x, odom.pose.pose.position.y)):
            speed = 0.2
        
        # if in speed zone 2
        if(self.inside_rectangle(0, 1, 2, 3, odom.pose.pose.position.x, odom.pose.pose.position.y)):
            speed = 0.1        
        return speed
    
    # hack: get current route to do line following, should be replaced when line detected is implemented.
    def check_current_route(self, odom):
        current_route = 0
        # route 0, middle up
        # route 1, left circle
        # route 2, right circle
        # route 3, park 1 line
        # route 4, park 1 circle
        # route 5, park 2 line
        # route 6, park 2 circle
        # route 7, park 3 line
        # route 8, park 3 circle
        # route 9, park 4 line
        # route 10, park 4 circle
        # route 11, park 5 line
        # route 12, park 5 circle
        # route 13, middle down
        return current_route
        
    def line_following(self):
        twist = Twist()

        while not rospy.is_shutdown():
            agv_odom = self.get_odom()
            park_command = self.get_park_command()
            current_marker = self.check_marker()
            current_speed = self.check_speed_zone(agv_odom)
            # hack: should be replaced when the marker detect module is implemented.
            current_route = self.check_current_route(agv_odom)
            if park_command == 0:
                    twist.linear.x = current_speed
                    if (current_route == 0):
                        current_side = self.check_side_of_line(route_0.start_point, route_0.end_point, agv_odom.pose.position)
                        if (current_side==RIGHT):
                            twist.angular.z = 0.15
                        elif (current_side==LEFT):
                            twist.angular.z = -0.15
                        else: 
                            twist.angular.z = 0
                    self._cmd_pub.publish(twist)
                    #rospy.loginfo('agv 0 is running')
            else:
                twist.linear.x = current_speed
                twist.angular.z = 0.0
                self._cmd_pub.publish(twist)
                turtlebot_moving = True
               

def main():
    rospy.init_node('agv_line_following')
    try:
        line_following = LineFollowing()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()