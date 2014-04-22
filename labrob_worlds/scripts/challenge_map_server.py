#!/usr/bin/env python
import rospy, math
from labrob_msgs.srv import GetMapPercentage

from nav_msgs.msg import OccupancyGrid

class ChallengeMapServer:
  def __init__(self):
    rospy.init_node('challenge_map_server')
    
    # Initial values
    size_m2 = rospy.get_param('/map_size_m2', 400.0)
    resolution = rospy.get_param('/map_resolution', 0.05)
    self.size_map = size_m2/resolution
    
    # Advertise the service
    s = rospy.Service('get_percent_map_completed', GetMapPercentage, self.handle_map)
    
    rospy.loginfo('Challenge begun. Map size: %d squares' % int(self.size_map));
  
  def handle_map(self, req):
    rospy.loginfo('Received map. Calculating Percentage Completed...');
    
    n_squares_mapped = sum(1 for x in req.map.data if x >= 0)
    percentage = float(n_squares_mapped)/float(self.size_map)
    
    rospy.loginfo('Map Complete: '+str(100.0 * percentage)+'%')

    return percentage
    
    
if __name__ == "__main__":
  ChallengeMapServer()
  rospy.spin()
