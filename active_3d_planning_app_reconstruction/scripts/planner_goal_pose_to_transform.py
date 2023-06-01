# ROS script to convert a goal pose to a transform
import rospy
import tf
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path

def callback(msg, topic):

    # Get the last pose in the path
    goal_pose = msg.poses[-1]

    # create transform
    t = TransformStamped()
    t.header.frame_id = "world"
    t.child_frame_id = topic
    t.transform.translation.x = goal_pose.pose.position.x
    t.transform.translation.y = goal_pose.pose.position.y
    t.transform.translation.z = goal_pose.pose.position.z
    t.transform.rotation.x = goal_pose.pose.orientation.x
    t.transform.rotation.y = goal_pose.pose.orientation.y
    t.transform.rotation.z = goal_pose.pose.orientation.z
    t.transform.rotation.w = goal_pose.pose.orientation.w

    # publish transform as a message using tranform_pubs
    publishing_on_topic = topic + '/transform'
    rospy.loginfo("publishing to " + str(publishing_on_topic))
    transform_pubs[publishing_on_topic].publish(t)

# main function
if __name__ == '__main__':
    # setup ros node
    rospy.init_node('planner_goal_pose_to_transform', anonymous=True)

    # get topic names from rosparam
    goal_pose_topics = rospy.get_param('/goal_topics', '')
    goal_pose_topics = goal_pose_topics.split(',')
    goal_pose_topics = [x.strip() for x in goal_pose_topics]

    rospy.loginfo("goal_pose_topics " + str(goal_pose_topics))

    # set up subscribers and publishers with callback that passes in message
    goal_pose_subs = {}
    transform_pubs = {}
    for topic in goal_pose_topics:
        goal_pose_subs[topic] = rospy.Subscriber(topic, Path, callback, topic, queue_size=1)
        publishing_on_topic = topic + '/transform'
        transform_pubs[publishing_on_topic] = rospy.Publisher(publishing_on_topic, TransformStamped, queue_size=5)

    # spin
    rospy.spin()

