#!/usr/bin/env python3

import rospy
import yaml
import tf

# from std_msgs.msg import String
from social_msgs.msg import Locals, Local
# from map.msg import StringArray
from map.srv import SaveLocal, SaveLocalResponse

ROS_RATE = 10
PUBLISHER_QUEUE_SIZE = 10

class Map:
    """docstring for map"""
    def __init__(self):

        self.rate = rospy.Rate(ROS_RATE)
        self.tb = tf.TransformBroadcaster()
        self.pub = rospy.Publisher('locals', Locals, queue_size=PUBLISHER_QUEUE_SIZE)
        rospy.Service('/map/save_local', SaveLocal, self.handle_start)

        fname = rospy.get_param("locals_file")


        while not rospy.is_shutdown():
            self.load_locals(fname)
            self.tb.sendTransform(
                (0,0,0),
                (0,0,0,1),
                rospy.Time.now(),
                "locals",
                "map")

            sa = Locals()
            for target in self.locals['locals']:
                loc = Local()
                loc.name = target
                loc.pose.position.x = self.locals['locals'][target][0][0]
                loc.pose.position.y = self.locals['locals'][target][0][1]
                loc.pose.position.z = 0 #self.locals['locals'][target][0][2]
                loc.pose.orientation.x = self.locals['locals'][target][1][0]
                loc.pose.orientation.y = self.locals['locals'][target][1][1]
                loc.pose.orientation.z = self.locals['locals'][target][1][2]
                loc.pose.orientation.w = self.locals['locals'][target][1][3]
                sa.locals.append(loc)

                self.tb.sendTransform(
                    (loc.pose.position.x, loc.pose.position.y, loc.pose.position.z),
                    (loc.pose.orientation.x,loc.pose.orientation.y,loc.pose.orientation.z,loc.pose.orientation.w),
                    rospy.Time.now(),
                    target,
                    "locals")


            self.pub.publish(sa)
            self.rate.sleep()

    def load_locals(self, fname):
        yaml_file = open(fname,'r')
        self.locals = yaml.safe_load(yaml_file)
        yaml_file.close()

    def save_locals(self, fname):
        yaml_file = open(fname,'w')
        yaml_file.write( yaml.dump(self.locals, default_flow_style=False))
        # self.locals = yaml.safe_load(yaml_file)
        yaml_file.close()

    def handle_start(self, req): 
        fname = rospy.get_param("locals_file")
        self.load_locals(fname)

        self.locals['locals'][req.name] = [
            [req.pose.position.x,
            req.pose.position.y],
            [req.pose.orientation.x,
            req.pose.orientation.y,
            req.pose.orientation.z,
            req.pose.orientation.w]]

        self.save_locals(fname)

        info = req.name + " saved!"
        rospy.loginfo(info)
        return SaveLocalResponse(True, info)

if __name__ == "__main__":
    rospy.init_node('map')
    Map()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

