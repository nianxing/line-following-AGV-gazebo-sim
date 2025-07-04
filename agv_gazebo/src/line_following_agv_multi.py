#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16


# Constant integers for directions
RIGHT = 1
LEFT = -1
ZERO = 0

# global parameters for zones, 
# bottom-left and top-right,(x1, y1, x2, y2)
# corners of rectangle.
Zone_1 = (-1.5, 0 , 3, 1.5) 
Zone_2 = (-1.5, -1.5, 3, 0)

# hack:
# global parameters for markers, which is in clockwise from 1~8
# global parameters for zones, 
# bottom-left and top-right,(x1, y1, x2, y2)
# corners of rectangle.
marker_1 = (-1, -2.05, -0.6, -1.95)
marker_2 = (-1, -1.05, -0.6, -0.95)
marker_3 = (-1, -0.05, -0.6, 0.05)
marker_4 = (-1, 0.95, -0.6, 1.05)
marker_5 = (-1, 1.95, -0.6, 2.05)

# hack:
# global parameters for route, should be replaced when line detection function is implemented.
# upline
route_0 = (-0.85, -2.5, -0.9, -2.5)
# bottom line
route_1 = (1.4, 2.5, 1.6, -2.5)
# left circle
route_2 = (0.375, -2.5, 1.22)
# right circle
route_3 = (0.375, 2.5, 1.22)
# park 1 line
route_4 = (-0.85, -1.0665, -2.7534, -1.0665)
# park 2
route_5 = (-0.85, -0.0665, -2.7534, -0.0665)
# park 3
route_6 = (-0.85, 0.9335, -2.7534, 0.9335)
# park 4
route_7 = (-0.85, 1.9335, -2.7534, 1.9335)
# park 5
route_8 = (-0.85, 2.9335, -2.7534, 2.9335)


Parking_0 = False
Parking_1 = False
Parking_2 = False
Parking_3 = False
Parking_4 = False

class LineFollowing():
    def __init__(self):
        self._cmd_pub_0 = rospy.Publisher('agv_0/cmd_vel', Twist, queue_size=1)
        self._cmd_pub_1 = rospy.Publisher('agv_1/cmd_vel', Twist, queue_size=1)
        self._cmd_pub_2 = rospy.Publisher('agv_2/cmd_vel', Twist, queue_size=1)
        self._cmd_pub_3 = rospy.Publisher('agv_3/cmd_vel', Twist, queue_size=1)
        self._cmd_pub_4 = rospy.Publisher('agv_4/cmd_vel', Twist, queue_size=1)
        self.line_following()
        
    def get_odom(self):
        current_odom_0 = rospy.wait_for_message('agv_0/odom', Odometry)
        current_odom_1 = rospy.wait_for_message('agv_1/odom', Odometry)
        current_odom_2 = rospy.wait_for_message('agv_2/odom', Odometry)
        current_odom_3 = rospy.wait_for_message('agv_3/odom', Odometry)
        current_odom_4 = rospy.wait_for_message('agv_4/odom', Odometry)
        return current_odom_0, current_odom_1,current_odom_2,current_odom_3,current_odom_4

    def get_park_command(self):
        park_command_0 = park_command_1 = park_command_2 = park_command_3 = park_command_4 = 0
        park_command_0 = rospy.wait_for_message('/park_agv_0', Int16)
        park_command_1 = rospy.wait_for_message('/park_agv_1', Int16)
        park_command_2 = rospy.wait_for_message('/park_agv_2', Int16)
        park_command_3 = rospy.wait_for_message('/park_agv_3', Int16)
        park_command_4 = rospy.wait_for_message('/park_agv_4', Int16)
        return park_command_0, park_command_1, park_command_2, park_command_3, park_command_4
    
    def in_rectangle(self,x1, y1, x2, y2, x, y):
        if (x > x1 and x < x2 and y > y1 and y < y2):
            return True
        else:
            return False
    
    def check_marker(self, odom):
        has_marker = False
        marker_id = -1
        # TODO: marker detector
        # check marker zone for parking
        if(self.in_rectangle(marker_1[0], marker_1[1], marker_1[2], marker_1[3], odom.pose.pose.position.x, odom.pose.pose.position.y)):
            has_marker = True
            marker_id = 1
        elif(self.in_rectangle(marker_2[0], marker_2[1], marker_2[2], marker_2[3], odom.pose.pose.position.x, odom.pose.pose.position.y)):
            has_marker = True
            marker_id = 2
        elif(self.in_rectangle(marker_3[0],marker_3[1], marker_3[2], marker_3[3], odom.pose.pose.position.x, odom.pose.pose.position.y)):
            has_marker = True
            marker_id = 3
        elif(self.in_rectangle(marker_4[0],marker_4[1], marker_4[2], marker_4[3], odom.pose.pose.position.x, odom.pose.pose.position.y)):
            has_marker = True
            marker_id = 4
        elif(self.in_rectangle(marker_5[0],marker_5[1], marker_5[2], marker_5[3], odom.pose.pose.position.x, odom.pose.pose.position.y)):
            has_marker = True
            marker_id = 5
        return has_marker, marker_id
    
    def in_circle(self, center_x, center_y, radius, x, y):
        square_dist = (center_x - x) ** 2 + (center_y - y) ** 2
        if (square_dist == radius ** 2):
            return 0
        elif (square_dist < radius ** 2):
            return 1
        else:
            return -1

    # point A start of line, point B end of line, P to be checked point
    def check_side_of_line(self, Ax, Ay, Bx, By, Px, Py):
        global RIGHT, LEFT, ZERO
        
        # Subtracting co-ordinates of
        # point A from B and P, to
        # make A as origin
        Bx -= Ax
        By -= Ay
        Px -= Ax
        Py -= Ay
    
        # Determining cross Product
        cross_product = Bx * Py - By * Px
    
        # Return RIGHT if cross product is positive
        if (cross_product > 0):
            return RIGHT
            
        # Return LEFT if cross product is negative
        if (cross_product < 0):
            return LEFT
    
        # Return ZERO if cross product is zero
        return ZERO

    
    def check_speed_zone(self, odom):
        speed = 0.5
        # if in speed zone 1
        if(self.in_rectangle(Zone_1[0], Zone_1[1], Zone_1[2], Zone_1[3], odom.pose.pose.position.x, odom.pose.pose.position.y)):
            speed = 0.2
        
        # if in speed zone 2
        if(self.in_rectangle(Zone_2[0], Zone_2[1], Zone_2[2], Zone_2[3], odom.pose.pose.position.x, odom.pose.pose.position.y)):
            speed = 0.1        
        return speed
    
    # hack: get current route to do line following, should be replaced when line detected is implemented.
    def check_current_route(self, odom):
        current_route = -1
        # route 0, middle up
        if (odom.pose.pose.position.x<0 and odom.pose.pose.position.y>-2.5 and odom.pose.pose.position.y<2.5):
            current_route = 0
        # route 1, middle down
        if (odom.pose.pose.position.x>0 and odom.pose.pose.position.y>-2.5 and odom.pose.pose.position.y<2.5):
            current_route = 1
        # route 2, left circle
        if (odom.pose.pose.position.y<-2.5):
            current_route = 2
        # route 3, right circle
        if (odom.pose.pose.position.y>2.5):
            current_route = 3
        
        #if (odom.pose.pose.position.x<-1)
        # route 4, park 1 
        # route 5, park 2 
        # route 6, park 3 
        # route 7, park 4 
        # route 8, park 5 
        return current_route

    def normal_drive(self, current_route, agv_odom):
        if (current_route == 0):
            #rospy.loginfo('current route is 0')
            #current_side = self.check_side_of_line(route_0[0], route_0[1], route_0[2], route_0[3],agv_odom.pose.pose.position.x, agv_odom.pose.pose.position.y)
            #rospy.loginfo('current_side: ')
            #rospy.loginfo(current_side)

            #if (current_side==RIGHT):
            if (agv_odom.pose.pose.position.x<-0.85):
                return 0.2, -0.1
            #elif (current_side==LEFT):
            elif (agv_odom.pose.pose.position.x>-0.85):
                return 0.2, 0.1
            else: 
                return 0.2, 0

        elif (current_route == 1):
            rospy.loginfo('current route is 1')
            #current_side = self.check_side_of_line(route_1[0], route_1[1], route_1[2], route_1[3], agv_odom.pose.pose.position.x, agv_odom.pose.pose.position.y)
            #if (current_side==RIGHT):
            if (agv_odom.pose.pose.position.x<1.46):
                return 0.2, 0.1
            #elif (current_side==LEFT):
            if (agv_odom.pose.pose.position.x>1.46):
                return 0.2, -0.1
            else: 
                return 0.2, 0
            
        elif (current_route == 2):
            rospy.loginfo('current route is 2')
            in_circle = self.in_circle(route_2[0], route_2[1], route_2[2], agv_odom.pose.pose.position.x, agv_odom.pose.pose.position.y)
            return 0.1, 0.1*in_circle

        elif (current_route == 3):
            rospy.loginfo('current route is 3')
            in_circle = self.in_circle(route_3[0], route_3[1], route_3[2], agv_odom.pose.pose.position.x, agv_odom.pose.pose.position.y)
            return 0.1, 0.1*in_circle

        return 0, 0
    
    def parking_drive(self, agv_odom, park_command):
        vel = 0.0
        steer = 0.0
        if(agv_odom.pose.pose.position.x>-2):
            if(park_command == 1):
                vel = 0.1
                if (agv_odom.pose.pose.position.x>=-1.5):
                    steer = 0.1
                elif (agv_odom.pose.pose.position.y>-1):
                    steer = 0.1
                elif (agv_odom.pose.pose.position.y<-1):
                    steer = -0.1
                else:
                    steer = 0
            elif(park_command == 2):
                vel = 0.1
                if (agv_odom.pose.pose.position.x>=-1.5):
                    steer = 0.1
                elif (agv_odom.pose.pose.position.y>0):
                    steer = -0.1
                elif (agv_odom.pose.pose.position.y<0):
                    steer = 0.1
                else:
                    steer = 0
            elif(park_command == 3):
                vel = 0.1
                if (agv_odom.pose.pose.position.x>=-1.5):
                    steer = 0.1
                elif (agv_odom.pose.pose.position.y>1):
                    steer = -0.1
                elif (agv_odom.pose.pose.position.y<1):
                    steer = 0.1
                else:
                    steer = 0
            elif(park_command == 4):
                vel = 0.1
                if (agv_odom.pose.pose.position.x>=-1.5):
                    steer = 0.1
                elif (agv_odom.pose.pose.position.y>2):
                    steer = 0.1
                elif (agv_odom.pose.pose.position.y<2):
                    steer = -0.1
                else:
                    steer = 0
            elif(park_command == 5):
                vel = 0.1
                if (agv_odom.pose.pose.position.x>=-1.5):
                    steer = 0.1
                elif (agv_odom.pose.pose.position.y>3):
                    steer = 0.1
                elif (agv_odom.pose.pose.position.y<3):
                    steer = -0.1
                else:
                    steer = 0
            return vel, steer 
        else:
            return vel, steer

    def drive_agv0(self,agv_odom, park_command):
        global Parking_0
        twist = Twist()
        current_route = self.check_current_route(agv_odom)  
        has_marker, current_marker = self.check_marker(agv_odom)
        if (park_command == 0):
            twist.linear.x, twist.angular.z = self.normal_drive(current_route, agv_odom)
            #rospy.loginfo('twist.angular.z: ')
            #rospy.loginfo(twist.angular.z)
            #rospy.loginfo('agv 0 is running on track')
        else:
            if(park_command == current_marker or Parking_0):
                #rospy.loginfo('in parking drive: ')
                #rospy.loginfo(park_command)
                twist.linear.x,twist.angular.z = self.parking_drive(agv_odom, park_command)
                Parking_0 = True
            else:
                twist.linear.x, twist.angular.z = self.normal_drive(current_route, agv_odom)
                
        self._cmd_pub_0.publish(twist)
    
    def drive_agv1(self,agv_odom, park_command):
        global Parking_1
        twist = Twist()
        current_route = self.check_current_route(agv_odom)  
        has_marker, current_marker = self.check_marker(agv_odom)
        if (park_command == 0):
            twist.linear.x, twist.angular.z = self.normal_drive(current_route, agv_odom)
            #rospy.loginfo('twist.angular.z: ')
            #rospy.loginfo(twist.angular.z)
            #rospy.loginfo('agv 0 is running on track')
        else:
            if(park_command == current_marker or Parking_1):
                #rospy.loginfo('in parking drive: ')
                #rospy.loginfo(park_command)
                twist.linear.x,twist.angular.z = self.parking_drive(agv_odom, park_command)
                Parking_1 = True
            else:
                twist.linear.x, twist.angular.z = self.normal_drive(current_route, agv_odom)
        self._cmd_pub_1.publish(twist)
    
    def drive_agv2(self,agv_odom, park_command):
        global Parking_2
        twist = Twist()
        current_route = self.check_current_route(agv_odom)  
        has_marker, current_marker = self.check_marker(agv_odom)
        if (park_command == 0):
            twist.linear.x, twist.angular.z = self.normal_drive(current_route, agv_odom)
            #rospy.loginfo('twist.angular.z: ')
            #rospy.loginfo(twist.angular.z)
            #rospy.loginfo('agv 0 is running on track')
        else:
            if(park_command == current_marker or Parking_2):
                #rospy.loginfo('in parking drive: ')
                #rospy.loginfo(park_command)
                twist.linear.x,twist.angular.z = self.parking_drive(agv_odom, park_command)
                Parking_2 = True
            else:
                twist.linear.x, twist.angular.z = self.normal_drive(current_route, agv_odom)
        self._cmd_pub_2.publish(twist)
    
    def drive_agv3(self,agv_odom, park_command):
        global Parking_3
        twist = Twist()
        current_route = self.check_current_route(agv_odom)  
        has_marker, current_marker = self.check_marker(agv_odom)
        if (park_command == 0):
            twist.linear.x, twist.angular.z = self.normal_drive(current_route, agv_odom)
            #rospy.loginfo('twist.angular.z: ')
            #rospy.loginfo(twist.angular.z)
            #rospy.loginfo('agv 0 is running on track')
        else:
            if(park_command == current_marker or Parking_3):
                #rospy.loginfo('in parking drive: ')
                #rospy.loginfo(park_command)
                twist.linear.x,twist.angular.z = self.parking_drive(agv_odom, park_command)
                Parking_3 = True
            else:
                twist.linear.x, twist.angular.z = self.normal_drive(current_route, agv_odom)
        self._cmd_pub_3.publish(twist)

    def drive_agv4(self,agv_odom, park_command):
        global Parking_4
        twist = Twist()
        current_route = self.check_current_route(agv_odom)  
        has_marker, current_marker = self.check_marker(agv_odom)
        if (park_command == 0):
            twist.linear.x, twist.angular.z = self.normal_drive(current_route, agv_odom)
            #rospy.loginfo('twist.angular.z: ')
            #rospy.loginfo(twist.angular.z)
            #rospy.loginfo('agv 0 is running on track')
        else:
            if(park_command == current_marker or Parking_4):
                #rospy.loginfo('in parking drive: ')
                #rospy.loginfo(park_command)
                twist.linear.x,twist.angular.z = self.parking_drive(agv_odom, park_command)
                Parking_4 = True
            else:
                twist.linear.x, twist.angular.z = self.normal_drive(current_route, agv_odom)
        print("agv 4")
        print(twist)
        self._cmd_pub_4.publish(twist)

    def line_following(self):
        park_command_0 = 0
        park_command_1 = 0
        park_command_2 = 0
        park_command_3 = 0
        park_command_4 = 0

        while not rospy.is_shutdown():
            agv_odom_0, agv_odom_1, agv_odom_2, agv_odom_3, agv_odom_4 = self.get_odom()
            #print(agv_odom_0)
            #park_command_0,park_command_1,park_command_2,park_command_3,park_command_4 = self.get_park_command()

            self.drive_agv0(agv_odom_0, park_command_0)
            self.drive_agv1(agv_odom_1, park_command_1)
            self.drive_agv2(agv_odom_2, park_command_2)
            self.drive_agv3(agv_odom_3, park_command_3)
            self.drive_agv4(agv_odom_4, park_command_4)


def main():
    rospy.init_node('agv_line_following')
    try:
        line_following = LineFollowing()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()