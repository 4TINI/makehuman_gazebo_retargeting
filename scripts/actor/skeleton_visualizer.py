#!/usr/bin/env python3

# import modules
import math
import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Point
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

def wait_for_tf(tf_buffer, parent_link, child_link):
    tf_found = False

    wait_for_tf_rate = rospy.Rate(30)
    while not tf_found and not rospy.is_shutdown():
        try:
            transform = tf_buffer.lookup_transform(parent_link, child_link, rospy.Time())
            tf_found = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            wait_for_tf_rate.sleep()
            continue
    
    return transform

class RealtimeVisualization():
    def __init__(self, ns, reference_frame, id_text_size, id_text_offset, skeleton_line_width, actor_name, color_index=1):
        self.ns = ns
        self.skeleton_frame = reference_frame
        self.id_text_size = id_text_size
        self.id_text_offset = id_text_offset
        self.skeleton_line_width = skeleton_line_width
        self.actor_name = actor_name

        # define a few colors we are going to use later on
        self.colors = [ColorRGBA(0.12, 0.63, 0.42, 1.00),
                       ColorRGBA(0.98, 0.30, 0.30, 1.00),
                       ColorRGBA(0.26, 0.09, 0.91, 1.00),
                       ColorRGBA(0.77, 0.44, 0.14, 1.00),
                       ColorRGBA(0.92, 0.73, 0.14, 1.00),
                       ColorRGBA(0.00, 0.61, 0.88, 1.00),
                       ColorRGBA(1.00, 0.65, 0.60, 1.00),
                       ColorRGBA(0.59, 0.00, 0.56, 1.00)]
        self.color = self.colors[color_index]

        self.body_parts_dict = {'Neck': None, 
                                'RightShoulder': 'RightArm', 
                                'RightElbow': 'RightForeArm', 
                                'RightWrist': 'RightHand',
                                'LeftShoulder': 'LeftArm',
                                'LeftElbow': 'LeftForeArm',
                                'LeftWrist': 'LeftHand',
                                'Hips': 'Hips',
                                'RightHip': 'RightUpLeg',
                                'RightKnee': 'RightLeg',
                                'RightAnkle': 'RightFoot',
                                'LeftHip': 'LeftUpLeg',
                                'LeftKnee': 'LeftLeg',
                                'LeftAnkle': 'LeftFoot'}
        
        self.upper_body_ids = [0, 7]
        self.hands_ids = [3, 2, 1, 0, 4, 5, 6]
        self.legs_ids = [10, 9, 8, 7, 11, 12, 13]
        self.body_parts = [self.upper_body_ids, self.hands_ids, self.legs_ids]
        # write actor id on the top of his head
        self.nose_id = 0

        # define a publisher to publish the 3D skeleton of multiple people
        self.skeleton_pub = rospy.Publisher(self.ns, MarkerArray, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # define a subscriber to retrive tracked bodies


    def create_marker(self, index, color, marker_type, size, time):
        '''
        Function to create a visualization marker which is used inside RViz
        '''
        marker = Marker()
        marker.id = index
        marker.ns = self.ns
        marker.color = color
        marker.action = Marker.ADD
        marker.type = marker_type
        marker.scale = Vector3(size, size, size)
        marker.pose.orientation.w = 1
        marker.header.stamp = time
        marker.header.frame_id = self.skeleton_frame
        marker.lifetime = rospy.Duration(0.5)  # 1 second
        return marker


    def update(self):
        '''
        We enter in a loop and wait for exit whenever `Ctrl + C` is pressed
        '''
        marker_counter = 0
        skeleton_counter = 0
        marker_array = MarkerArray()
        
        now = rospy.Time.now()
        marker_color = self.color
        body_marker = [self.create_marker(marker_counter + idx, marker_color, Marker.LINE_STRIP, self.skeleton_line_width, now) for idx in range(len(self.body_parts))]
        
        
        
        marker_counter += len(self.body_parts)
        points_list = []
        for id, keypoint in enumerate(self.body_parts_dict):
            pt = Point()

            if keypoint == 'Neck':
                groundtruth_transform_stamped_left_collar = wait_for_tf(self.tf_buffer, "world", self.actor_name+'_LeftShoulder')

                groundtruth_transform_stamped_right_collar = wait_for_tf(self.tf_buffer, "world", self.actor_name+'_RightShoulder')
                
                pt.x = (groundtruth_transform_stamped_left_collar.transform.translation.x+groundtruth_transform_stamped_right_collar.transform.translation.x)/2
                pt.y = (groundtruth_transform_stamped_left_collar.transform.translation.y+groundtruth_transform_stamped_right_collar.transform.translation.y)/2
                pt.z = (groundtruth_transform_stamped_left_collar.transform.translation.z+groundtruth_transform_stamped_right_collar.transform.translation.z)/2
            else:
                groundtruth_transform_stamped = wait_for_tf(self.tf_buffer, "world", actor_name + "_" + self.body_parts_dict[keypoint])
                pt.x = groundtruth_transform_stamped.transform.translation.x
                pt.y = groundtruth_transform_stamped.transform.translation.y
                pt.z = groundtruth_transform_stamped.transform.translation.z

            points_list.append(pt)

        for index, body_part in enumerate(self.body_parts):
            body_marker[index].points = [points_list[idx] for idx in body_part]

        marker_array.markers.extend(body_marker)
        skeleton_id = self.create_marker(marker_counter, marker_color, Marker.TEXT_VIEW_FACING, self.id_text_size, now)
        marker_counter += 1
        skeleton_id.text = str(skeleton_counter)
        skeleton_counter += 1
        self.skeleton_pub.publish(marker_array)

if __name__ == '__main__':
    # define some constants
    ns = 'visualization'

    # initialize ros node
    rospy.init_node('visualizer_node', anonymous=False)

    # read the parameters from ROS parameter server
    reference_frame = rospy.get_param('~reference_frame', 'world')
    actor_name = rospy.get_param('~model_name', 'actor')
    id_text_size = rospy.get_param('~id_text_size', 0.3)
    id_text_offset = rospy.get_param('~id_text_offset', 0)
    skeleton_line_width = rospy.get_param('~skeleton_line_width', 0.01)
    skeleton_color_index = rospy.get_param('~color_index', 1)
    publishing_rate = rospy.get_param('~publishing_rate', 100.0)

    rate = rospy.Rate(publishing_rate)

    # instantiate the RealtimeVisualization class
    visualization = RealtimeVisualization(ns, reference_frame, id_text_size, id_text_offset, skeleton_line_width, actor_name, skeleton_color_index)
    
    while not rospy.is_shutdown():
        visualization.update()
        
        rate.sleep()