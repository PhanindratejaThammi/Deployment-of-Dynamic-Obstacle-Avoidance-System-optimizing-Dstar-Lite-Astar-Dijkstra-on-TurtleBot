import rospy
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseAction, MoveBaseGoal
import actionlib

# Initialize action client outside the send_goal function
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

def send_goal(x, y):
    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0  # No rotation of the mobile base frame w.r.t. map frame

    # Sends the goal to the action server.
    print("Sending goal:", goal)
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    wait_4_result = client.wait_for_result()

    # If the result doesn't arrive, assume the Server is not available
    if not wait_4_result:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available! ---> shutdown")
    else:
        result = client.get_result()
        print("Goal result:", result)
        return result  # Result of executing the action

if __name__ == '__main__':
    rospy.init_node('goal_generate_py')
    client.wait_for_server()
    send_goal(3.89, -5.23)

    # try:
    #     # Wait for the action server to become available
    #     client.wait_for_server()
    #     send_goal(3.89, -5.2)
    #     rospy.loginfo("Goal execution done!")
    # except rospy.ROSInterruptException:
    #     rospy.loginfo("Navigation test finished.")
