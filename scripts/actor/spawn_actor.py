#!/usr/bin/env python3
import rospy
from actor import GazeboActor

def main():
    rospy.init_node('actor_spawner', anonymous=True)
    
    actor_name=rospy.get_param('~actor_name')
    anim_name='demo'
    
    path=rospy.get_param('~actor_path')
    
    x = rospy.get_param('~actor_init_x')
    y = rospy.get_param('~actor_init_y')
    z = rospy.get_param('~actor_init_z')
    
    roll = rospy.get_param('~actor_init_roll')
    pitch = rospy.get_param('~actor_init_pitch')
    yaw = rospy.get_param('~actor_init_yaw')
    
    init_pose = [x, y, z, roll, pitch, yaw]
    
    collision_file = rospy.get_param('~collision_file')
        
    my_actor = GazeboActor(actor_name, anim_name, path, init_pose, collision_file)
    my_actor.spawn_actor()
      
if __name__ == "__main__":
    main()