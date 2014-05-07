#!/usr/bin/env python
import rospy, rosbag, math, datetime, csv, os

# Messages
from geometry_msgs.msg import Twist
from labrob_msgs.srv import GetMapPercentage, GetMapPercentageRequest
from labrob_msgs.msg import StatusMessage
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int32, Bool

class ChallengeMapServer:
  def __init__(self):
    rospy.init_node('challenge_server')
    # Read parameters from the server
    self.map_size_m2 = float(rospy.get_param('/map_size_m2', 400.0))
    # Initial values
    self.start_time = None
    self.using_teleop = False
    self.percentage_mapped = 0    
    # Advertise the service
    s = rospy.Service('get_percent_map_completed', GetMapPercentage, self.handle_map)
    # Create the log files where the data will be saved
    filename = self.get_filename()
    self.bag_writer = rosbag.Bag(filename + '.bag', 'w')
    self.csv_file = open(filename + '.csv', 'wb')
    self.csv_writer = csv.writer(self.csv_file, delimiter='\t', quotechar='"')
    self.csv_writer.writerow(['course_completed','time (s.)','coins_collected','total_coins','percentage_mapped','teleop_time (s.)'])
    # Setup subscribers / publishers
    rospy.Subscriber('/ed27fa22584967f31278es75efd0e16', StatusMessage, self.status_cb)
    rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_cb)
    rospy.Subscriber('/using_telop', Bool, self.using_telop_cb)
    rospy.Subscriber('/map', OccupancyGrid, self.map_cb)
    # Output to the console
    rospy.loginfo('Map size: %f m2' % self.map_size_m2)
    rospy.loginfo('Waiting for first msg on [/cmd_vel] to start')
    # Register rospy shutdown hook
    rospy.on_shutdown(self.shutdown_hook)
    # Teleop time estimation at 100 Hz
    self.timer_rate = 100.0
    self.timer = rospy.Timer(rospy.Duration(1.0 / self.timer_rate), self.teleop_timing)    
    self.teleop_time = 0
    
  def map_cb(self, msg):
    self.percentage_mapped = self.calculate_percentage(msg)
    
  def teleop_timing(self, event):
    if self.using_teleop:
      self.teleop_time += 1.0 / self.timer_rate
    
  def using_telop_cb(self, msg):
    self.using_teleop = msg.data
    
  def status_cb(self, msg):
    if not self.start_time == None:
      self.bag_writer.write('coins_remaining', Int32(msg.total - msg.collected))
      self.bag_writer.write('course_complete', Bool(msg.complete))
      totalsecs = '%.1f' % (rospy.get_rostime() - self.start_time).to_sec()
      self.csv_writer.writerow([msg.complete, totalsecs, msg.collected, msg.total, self.percentage_mapped, '%.1f' % self.teleop_time])      
    
  def cmd_vel_cb(self, msg):
    if self.start_time == None:
      rospy.loginfo('Challenge begun! Good luck!')
      self.start_time = rospy.get_rostime()
    self.bag_writer.write('cmd_vel', msg)
  
  def handle_map(self, req):
    rospy.loginfo('Received map. Calculating Percentage Completed...');
    percentage = self.calculate_percentage(req.map)
    rospy.loginfo('Map Complete: '+str(100.0 * percentage)+'%')
    self.bag_writer.write('map', req.map)
    return percentage
  
  def calculate_percentage(self, map_msg):
    n_squares_mapped = float(sum(1 for x in map_msg.data if x >= 0))
    resolution = map_msg.info.resolution
    total_squares = float(self.map_size_m2 / (resolution ** 2))
    percentage = n_squares_mapped / total_squares
    return percentage
    
  
  def shutdown_hook(self):
    self.timer.shutdown()
    self.bag_writer.close()
  
  def get_filename(self):
    home = os.path.expanduser('~')
    logpath = home + '/.ros/maze_log/'
    if not os.path.exists(logpath):
      os.makedirs(logpath)
    return logpath + datetime.datetime.now().strftime('%y-%m-%d-%H-%M')
    
if __name__ == '__main__':
  ChallengeMapServer()
  rospy.spin()
