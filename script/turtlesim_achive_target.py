import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

def init_node():
    rospy.init_node('turtlebot_target_controller', anonymous=True)
    # Target publisher
    target_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    # Subscriber to catch the currect position
    pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, update_pose)
    rate = rospy.Rate(10)
    return target_publisher, rate

def update_pose(data):
    # Get the value in topic t see the current cordinates
    # Get the x and y cordinates of the position - Will be assigned to global variable
    global pose
    pose = data
    pose.x = round(pose.x, 4)
    pose.y = round(pose.y, 4)

def direct_distance(target_pose):
    # Calculate the distance between target and current positions - Pythagorean theorem
    return sqrt(pow((target_pose.x - pose.x), 2) + pow((target_pose.y - pose.y), 2))

def linear_vel(target_pose, constant=1.5):
    # Calculate the speed to travel to target
    # Used constant value to determine the speed depend on the 
    # distance (Close target - Slow and rest high speed)
    return constant * direct_distance(target_pose)

def steering_angle(target_pose):
    # Calculate theta value
    return atan2(target_pose.y - pose.y, target_pose.x - pose.x)

def angular_vel(target_pose, constant=6):    
    # Calculate the speed to travel to change the angle
    # Used constant value to determine the speed depend on the distance
    return constant * (steering_angle(target_pose) - pose.theta)

def goToTarget():
    try:
        target_publisher, rate = init_node()
        
        target_pose = Pose()
        target_pose.x = float(input("Enter the x goal: "))
        target_pose.y = float(input("Enter the y goal: "))
        distance_tolerance = float(input("Enter the tolerance: "))

        vel_msg = Twist()

        while direct_distance(target_pose) >= distance_tolerance:
            vel_msg.linear.x = linear_vel(target_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = angular_vel(target_pose)

            target_publisher.publish(vel_msg)
            rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        target_publisher.publish(vel_msg)

        rospy.spin()
    
    except Exception as e:
        print(e)

    finally:
        # Stop the turtle when exiting
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        target_publisher.publish(vel_msg)


if __name__ == '__main__':
    try:
        goToTarget()
    except rospy.ROSInterruptException:
        pass
