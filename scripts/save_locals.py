#!/usr/bin/env python3

import rospy
import yaml
import tf

from geometry_msgs.msg import Pose, PoseStamped, PoseArray

class SaveLocals:
  """docstring for SaveLocals"""

  def __init__(self):

    rospy.Subscriber("move_base_simple/goal", PoseStamped, self.callback)

    print ("\nPLEASE, SEND A GOAL WITH NAV 2D GOAL IN GRAPHIC INTERFACE TO SAVE A POINT!\n")

    while not rospy.is_shutdown():
      fname = rospy.get_param("/locals_file")
      yaml_file = open(fname,'r')
      yaml_data = yaml.load(yaml_file)
      keys = yaml_data['locals'].keys()
      for local in keys:

        br = tf.TransformBroadcaster()
        br.sendTransform(
          (yaml_data['locals'][local][0][0],yaml_data['locals'][local][0][1],0),
          (yaml_data['locals'][local][1][0],yaml_data['locals'][local][1][1],
           yaml_data['locals'][local][1][2],yaml_data['locals'][local][1][3]),
          rospy.Time.now(),
          local,
          "map")

  def callback(self, data):
    fname = rospy.get_param("/locals_file")
    yaml_file = open(fname,'r')
    yaml_data = yaml.load(yaml_file)
    keys = yaml_data['locals'].keys()
    local = raw_input("Select a location or enter a new one. \n(options: "+str(keys)+") ")

    if [x for x in keys if x == local]:
        yaml_data['locals'][local] = [[data.pose.position.x,data.pose.position.y],[data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w]]
        yaml_file = open(fname,'w')
        yaml_file.write( yaml.dump(yaml_data, default_flow_style=False))
        print ("Point Saved!")
    else:
        c = raw_input("Save a new place? (y/n)")
        if c == 'y':
          yaml_data['locals'][local] = [[data.pose.position.x,data.pose.position.y],[data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w]]
          yaml_file = open(fname,'w')
          yaml_file.write( yaml.dump(yaml_data, default_flow_style=False))
          print ("Point Saved!")
        else:
          print ("Point not saved!")
   
    print ("\nPLEASE, SEND A GOAL WITH NAV 2D GOAL IN GRAPHIC INTERFACE TO SAVE A POINT!\n")

if __name__ == "__main__":
    rospy.init_node('SaveLocals')
    SaveLocals()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
