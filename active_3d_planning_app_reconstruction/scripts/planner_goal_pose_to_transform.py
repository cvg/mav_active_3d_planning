# ROS script to convert a goal pose to a transform
import rospy
import tf
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path

def callback(msg, callback_args):
    # get topic and frame_id from callback_args
    topic = callback_args[0]
    frame_id = callback_args[1]

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
    transform_pubs[publishing_on_topic].publish(t)

# main function
if __name__ == '__main__':
    # setup ros node
    rospy.init_node('planner_goal_pose_to_transform')

    # get topic names from parameter server, should be the paths
    goal_pose_topics = ['/planner/command/trajectory_path'] # TODO: change for multi-agent
    goal_pose_frame_id_dict = {'/planner/command/trajectory_path': 'anymal'} # TODO: change for multi-agent

    # set up subscribers and publishers with callback that passes in message
    goal_pose_subs = {}
    transform_pubs = {}
    for topic in goal_pose_topics:
        goal_pose_subs[topic] = rospy.Subscriber(topic, Path, callback, callback_args=(topic, goal_pose_frame_id_dict[topic]), queue_size=1)
        publishing_on_topic = topic + '/transform'
        transform_pubs[publishing_on_topic] = rospy.Publisher(publishing_on_topic, TransformStamped, queue_size=5)

    # spin
    rospy.spin()

