#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import LinkStates
import geometry_msgs.msg
import tf2_ros

def gazebo_links_callback(msg):
    global gazebo_links, gazebo_links_receiving_time
    gazebo_links = msg
    gazebo_links_receiving_time = rospy.Time.now().to_sec()

if __name__ == '__main__':
    try:
        rospy.init_node('gazebo_model_tf_publisher')

        gazebo_links = LinkStates()
        gazebo_links_receiving_time = 0
        gazebo_links_timeout = 0.5

        publishing_rate = rospy.get_param("~publishing_rate", 30)
        model_name = rospy.get_param("~model_name", "Actor")
        reference_frame = rospy.get_param("~reference_frame", "world")

        gazebo_link_state = rospy.Subscriber('/gazebo/link_states', LinkStates, gazebo_links_callback)

        tf_broadcaster = tf2_ros.TransformBroadcaster()

        rate = rospy.Rate(publishing_rate)

        prev_broadcast_time = rospy.Time.now().to_sec()
        broadcast_time_sleep = 0.001

        # Wait for ros time now
        while not rospy.Time.now().to_sec() and not rospy.is_shutdown():
            rate.sleep()

        rospy.loginfo("Gazebo model tf publisher initialized.")

        while not rospy.is_shutdown():
            actual_time = rospy.Time.now().to_sec()
            if (actual_time - gazebo_links_receiving_time) < gazebo_links_timeout:
                if (actual_time - prev_broadcast_time) > broadcast_time_sleep:
                    transforms_stamped = []
                    for link_cnt, link_name in enumerate(gazebo_links.name):
                        if model_name == link_name.rpartition('::')[0]:
                            transform_stamped = geometry_msgs.msg.TransformStamped()
                            transform_stamped.header.stamp = rospy.Time.now()
                            transform_stamped.header.frame_id = reference_frame
                            transform_stamped.child_frame_id = link_name.replace("::", "_")
                            transform_stamped.transform.translation.x = gazebo_links.pose[link_cnt].position.x
                            transform_stamped.transform.translation.y = gazebo_links.pose[link_cnt].position.y
                            transform_stamped.transform.translation.z = gazebo_links.pose[link_cnt].position.z
                            transform_stamped.transform.rotation = gazebo_links.pose[link_cnt].orientation
                            transforms_stamped.append(transform_stamped)
                    tf_broadcaster.sendTransform(transforms_stamped)
                    prev_broadcast_time = actual_time
            else:
                rospy.logwarn_throttle(10, "Waiting for \"/gazebo/link_states\" message...")

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
