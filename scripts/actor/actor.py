import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel

class GazeboActor:
     
    def __init__(self, actor_name, path, pose, collision_file, loop_flag):
        self.actor_name = actor_name
        self.path = path    
        self.pose = ' '.join(str(e) for e in pose)
        self.collision_file = collision_file
        self.loop_flag = loop_flag
        
        # self.model_xml_text = "<?xml version=\"1.0\" ?><sdf version=\"1.6\"><world name=\"default\"><actor name=\""+self.actor_name+"\"><skin><filename>"+self.path+"</filename></skin><animation name=\""+self.anim_name+"\"><filename>"+self.path+"</filename></animation><script><trajectory id=\"0\" type=\"falling\"><waypoint><time>100</time><pose>"+self.pose+"</pose></waypoint></trajectory></script></actor></world></sdf>"
        # self.model_xml_text = "<?xml version=\"1.0\" ?><sdf version=\"1.6\"><world name=\"default\"><actor name=\""+self.actor_name+"\"><skin><filename>"+self.path+"</filename></skin><animation name=\""+self.anim_name+"\"><filename>"+self.path+"</filename><scale>1.000000</scale><interpolate_x>true</interpolate_x></animation></actor></world></sdf>"
        f = open(self.collision_file,'r')
        sdff = f.read()
        
        self.model_xml_text = "<?xml version=\"1.0\" ?><sdf version=\"1.6\"><actor name=\""+self.actor_name+"\">"+sdff+"<skin><filename>"+self.path+"</filename></skin><script><loop>"+str(self.loop_flag)+"</loop></script></actor></sdf>"
    
    def spawn_actor(self):

        try:
            rospy.wait_for_service('/gazebo/spawn_sdf_model')
            spawn_sdf_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)       
            
            resp = spawn_sdf_client(model_name=self.actor_name, 
                                model_xml=self.model_xml_text, 
                                robot_namespace="",
                                reference_frame="world")
            
            rospy.loginfo("Spawn result: "+str(resp.success)+" - "+resp.status_message)
            
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

    def delete_actor(self):  
        
        try:
            delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            resp_delete = delete_model_client(self.actor_name)
            rospy.loginfo("Delete result: "+str(resp_delete.success)+" - "+resp_delete.status_message)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)