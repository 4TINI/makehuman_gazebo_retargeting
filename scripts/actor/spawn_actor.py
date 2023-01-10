#!/usr/bin/env python3
import rospy
from actor import GazeboActor
import tf2_ros

def get_pose(tf_buffer, parent_frame, child_frame):
    pose = None
    try:
        pose = tf_buffer.lookup_transform(parent_frame, child_frame, rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("No transform found. \""+parent_frame+"\" and \""+child_frame+"\"")
        pass

    return pose

def main():
    rospy.init_node('actor_spawner', anonymous=True)
    
    actor_name=rospy.get_param('~actor_name') + str(rospy.get_param('~actor_number'))
    
    path=rospy.get_param('~actor_path')
    
    x = rospy.get_param('~actor_init_x')
    y = rospy.get_param('~actor_init_y')
    z = rospy.get_param('~actor_init_z')
    
    roll = rospy.get_param('~actor_init_roll')
    pitch = rospy.get_param('~actor_init_pitch')
    yaw = rospy.get_param('~actor_init_yaw')
    
    init_pose = [x, y, z, roll, pitch, yaw]
    
    collision_file = rospy.get_param('~collision_file')
    loop_flag = rospy.get_param('~animation_loop')
        
    my_actor = GazeboActor(actor_name, path, init_pose, collision_file, loop_flag)
    
    my_actor.spawn_actor()

    tfBuffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tfBuffer)
    
    rospy.sleep(1)

    prev_pose = get_pose(tfBuffer, 'world', actor_name + '_Hips')    
    rate = rospy.Rate(30) # 10hz
    
    if not loop_flag:
        while not rospy.is_shutdown():  
            rate.sleep()          
            next_pose = get_pose(tfBuffer, 'world', actor_name + '_Hips')
            
            if next_pose.transform.translation.x == prev_pose.transform.translation.x and \
                next_pose.transform.translation.y == prev_pose.transform.translation.y and \
                next_pose.transform.translation.z == prev_pose.transform.translation.z:
                my_actor.delete_actor()
                break
            else:
                prev_pose = next_pose
            
      
if __name__ == "__main__":
    main()