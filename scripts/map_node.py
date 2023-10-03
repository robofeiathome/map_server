#!/usr/bin/env python3

import rospy
import yaml
import tf
from social_msgs.msg import Locals, Local
from map.srv import ReloadLocals

# Constants
ROS_RATE = 10
PUBLISHER_QUEUE_SIZE = 10
Z_POSITION_DEFAULT = 0

class MapHandler:
    """Class responsible for managing and broadcasting local map data."""
    
    def __init__(self):
        """Initialize the MapHandler."""
        self.initialize_ros_components()
        filename = rospy.get_param("locals_file")
        self.load_locals(filename)
        self.run_main_loop()

    def initialize_ros_components(self):
        """Initialize ROS components like publishers and broadcasters."""
        self.rate = rospy.Rate(ROS_RATE)
        self.transform_broadcaster = tf.TransformBroadcaster()
        self.publisher = rospy.Publisher('locals', Locals, queue_size=PUBLISHER_QUEUE_SIZE)
        rospy.Service('/map_handler/reload_locals', ReloadLocals, self.handle_reload_locals_request)

    def handle_reload_locals_request(self, request):
        """Handle requests to reload locals from the file."""
        try:
            filename = rospy.get_param("locals_file")
            self.load_locals(filename)
            return True, "Locals reloaded successfully."
        except Exception as e:
            rospy.logerr(f"Failed to reload locals: {str(e)}")
            return False, f"Failed to reload locals: {str(e)}"


    def load_locals(self, filename):
        """Load local map data from a given YAML file."""
        try:
            with open(filename, 'r') as yaml_file:
                self.locals_data = yaml.safe_load(yaml_file)
        except FileNotFoundError:
            rospy.logwarn(f"File {filename} not found. Using default data.")
            self.locals_data = {'locals': {}}
        except Exception as e:
            rospy.logerr(f"Error reading from {filename}: {str(e)}. Using default data.")
            self.locals_data = {'locals': {}}

    def run_main_loop(self):
        """Main loop to broadcast and publish local map data."""
        while not rospy.is_shutdown():
            self.broadcast_transforms()
            self.publish_local_data()
            self.rate.sleep()

    def broadcast_transforms(self):
        """Broadcast transforms for local map data."""
        self.transform_broadcaster.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "locals", "map")
        for target, values in self.locals_data['locals'].items():
            position, orientation = values
            self.transform_broadcaster.sendTransform(
                (position[0], position[1], Z_POSITION_DEFAULT),
                tuple(orientation),
                rospy.Time.now(),
                target,
                "locals"
            )

    def publish_local_data(self):
        """Publish local map data using a ROS publisher."""
        local_array = Locals()
        for target, values in self.locals_data['locals'].items():
            local_item = Local()
            position, orientation = values
            local_item.name = target
            local_item.pose.position.x, local_item.pose.position.y = position
            local_item.pose.position.z = Z_POSITION_DEFAULT
            local_item.pose.orientation.x, local_item.pose.orientation.y, local_item.pose.orientation.z, local_item.pose.orientation.w = orientation
            local_array.locals.append(local_item)
        self.publisher.publish(local_array)

if __name__ == "__main__":
    """Main execution point for the map handler node."""
    rospy.init_node('map_handler')
    MapHandler()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
