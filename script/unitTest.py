import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

def init_node():
    rospy.init_node('turtlebot_controller', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, update_pose)
    rate = rospy.Rate(10)
    return velocity_publisher, rate

def update_pose(data):
    """Callback function which is called when a new message of type Pose is received by the subscriber."""
    global pose
    pose = data
    pose.x = round(pose.x, 4)
    pose.y = round(pose.y, 4)

def euclidean_distance(goal_pose):
    """Euclidean distance between current pose and the goal."""
    return sqrt(pow((goal_pose.x - pose.x), 2) + pow((goal_pose.y - pose.y), 2))

def linear_vel(goal_pose, constant=1.5):
    """Calculates linear velocity towards the goal."""
    return constant * euclidean_distance(goal_pose)

def steering_angle(goal_pose):
    """Calculates the steering angle towards the goal."""
    return atan2(goal_pose.y - pose.y, goal_pose.x - pose.x)

def angular_vel(goal_pose, constant=6):
    """Calculates angular velocity towards the goal."""
    return constant * (steering_angle(goal_pose) - pose.theta)

def move2goal():
    try:
        """Moves the turtle to the goal."""
        velocity_publisher, rate = init_node()
        
        goal_pose = Pose()
        goal_pose.x = float(input("Set your x goal: "))
        goal_pose.y = float(input("Set your y goal: "))
        distance_tolerance = float(input("Set your tolerance: "))

        vel_msg = Twist()

        while euclidean_distance(goal_pose) >= distance_tolerance:
            vel_msg.linear.x = linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = angular_vel(goal_pose)

            velocity_publisher.publish(vel_msg)
            rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)

        rospy.spin()
    
    except Exception as e:
        print(e)

    finally:
        # Stop the turtle when exiting
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    try:
        move2goal()
    except rospy.ROSInterruptException:
        pass
