#To handle ROS node
import rclpy


#ROS image message
from std_msgs.msg import Empty


#Node base to be able to integrate our project to the Tello_ws
from plugin_server_base.plugin_base import PluginBase, NodeState

image_raw = "/camera/image_raw"

class Test1(PluginBase):

    
    def __init__(self,name):
        #Creating the Node
        super().__init__(name)
        self.get_logger().info("Arrived 2")
        self.i = 0
        self.takeoff = "/takeoff"
        self.publisher_takeoff = self.create_publisher(Empty,self.takeoff,1)
    
    

######################## Publisher #####################################################################################  
    
        
    def tick(self) -> NodeState:
        """This method is a mandatory for PluginBase node. It defines what we want our node to do.
        It gets called 20 times a second if state=RUNNING
        """
        if self.i%30 == 0:
            print("Publishing takeoff")
            self.publisher_takeoff.publish(Empty())
            
            self.i +=1
        return NodeState.RUNNING

def main(args=None):
    #Intialization ROS communication 
    print("Arrived 6")

    rclpy.init(args=args)
    test = Test1('all')

    test.get_logger().info("Arrived 7")
    #execute the callback function until the global executor is shutdown
    rclpy.spin(test)

   
    #destroy the node. It is not mandatory, since the garbage collection can do it
    test.destroy_node()
    
    
    rclpy.shutdown()        
