#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


import rclpy.logging







class PosePlugin(Node):
  """
    @class PosePlugin
    @brief A plugin for managing and serving target poses and sizes within a robot world model.

    This class provides services to retrieve the position and orientation of targets,
    based on configuration parameters or Redis database entries.
  """

  def __init__(self):
    """
    @brief Constructor for PosePlugin.
    @param node_name The name of the ROS2 node.
    """  
    super().__init__('pose_plugin')

    self.declareParameters()
    teste = self.readParameter()
    self.get_logger().info(f'Valor TESTE: {type(teste)}')
    self.get_logger().info('PosePlugin initialized ' + teste)
    
  def declareParameters(self):
    """
    @brief Declares the initial list of target keys as parameters.
    """
    self.declare_parameter('teste', 'LIGOU')
  
  
  def readParameter(self):
    """
    @brief Reads the list of target keys from declared parameters.
    @return A list of target names.
    """
    return self.get_parameter('teste').value
  
  


def main(args=None) -> None: 
    rclpy.init(args=args)
    node = PosePlugin()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()