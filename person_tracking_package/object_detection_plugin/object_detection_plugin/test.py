################################### Imports #######################################

#for handling ROS node
import rclpy
from rclpy.node import Node

import py_trees


class Test(Node):

   

    def __init__(self,name):

        #Creating the Node
        super().__init__(name)

    def test_func(self):
        blackboard1 = py_trees.blackboard.Client(name="test1")
        blackboard2 = py_trees.blackboard.Client(name="test2")

        blackboard1.register_key(key="foo", access=py_trees.common.Access.WRITE)
        #print(blackboard1.foo)
        blackboard1.foo = "leila"
        print(blackboard1.foo)
        blackboard2.register_key(key="foo",access=py_trees.common.Access.READ)
        print(blackboard2.foo)
        


          

    
def main(args=None):
    #Initialization of ROS communication  
    rclpy.init(args=args)

    #Node instantiation
    test_node = Test('test')

    #execute the callback function until the global executor is shutdown
    #rclpy.spin(test_node)
    test_node.test_func()

    #destroy the node. It is not mandatory, since the garbage collection can do it
    test_node.destroy_node()
    
    rclpy.shutdown()        
