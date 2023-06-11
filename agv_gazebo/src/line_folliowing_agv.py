import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2

class LineFollowing():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('agv_0/cmd_vel', Twist, queue_size=1)
        self.line_following()
        
    def get_odom(self):
        current_odom = rospy.wait_for_message('agv_0/odom', Odometry)
        return current_odom

    def get_rest_command(self):
        rest_command = rospy.wait_for_message('agv_0/odom', Odometry)
        return rest_command
    
    def check_speed_zone(self):
        speed = 0
        return speed
    
    def check_side(self, odom):
        side = 0
        # 1. get geometry from odom

        # 2. calculate side
        return side
        
    def line_following(self):
        twist = Twist()
        turtlebot_moving = True

        while not rospy.is_shutdown():
            agv_odom = self.get_odom()
            min_distance = min(lidar_distances)

            if min_distance < SAFE_STOP_DISTANCE:
                if turtlebot_moving:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self._cmd_pub.publish(twist)
                    turtlebot_moving = False
                    rospy.loginfo('Stop!')
            else:
                twist.linear.x = LINEAR_VEL
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