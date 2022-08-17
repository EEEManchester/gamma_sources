#! /usr/bin/env python

# General Dependancies
import rospy
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
# import tf.transformations as tft
# from dynamic_reconfigure.server import Server
# import tf2_ros as tf2




class gammaSpawner():
    def __init__(self):
        # Get parameters
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.n_sources = rospy.get_param("~n_sources", 3)
        self.scale = rospy.get_param("~scale", 0.2)

        # Marker Handling
        self.markerserver = InteractiveMarkerServer("gamma_sources")
        self.menu_handler = MenuHandler()

        self.markerposes = PoseArray()
        self.markerposes.header.frame_id = self.global_frame

        self.intensity = []

        # self.marker_pub = rospy.Publisher("/gamma_marker", Marker, queue_size = 0)
        self.pose_pub = rospy.Publisher(rospy.get_name() +"/poses", PoseArray, latch = True, queue_size=1)

        for source in range(self.n_sources):
            name_id = "source_"+str(source)
            position = Point(source, 0, 0)
            self.makeMarker(position, name_id)
            # rospy.set_param(name_id+'/pose', {'x': position.x, 'y': position.y, 'z': position.z})
            # self.intensitydict[name_id] = 100
            self.intensity.append(100)
            pose = Pose()
            pose.position = position
            pose.orientation = Quaternion()
            self.markerposes.poses.append(pose)

        rospy.set_param(rospy.get_name()+"/intensity", self.intensity)
        self.markerserver.applyChanges()

        # Ensure poses are published before any movement
        self.markerposes.header.stamp = rospy.Time.now()
        self.pose_pub.publish(self.markerposes)



    def makeMarker(self, position, name_id):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.global_frame
        int_marker.pose.position = position
        int_marker.scale = self.scale
        int_marker.name = name_id
        int_marker.description = name_id

        self.makeBoxControl(int_marker)
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_3D


        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        self.markerserver.insert(int_marker, self.processFeedback)
        self.menu_handler.apply( self.markerserver, int_marker.name )



    def makeBoxControl(self, markerMsg ):
        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append( self.makeBox(markerMsg) )
        markerMsg.controls.append( control )

        return control

    def makeBox(self, msg):
        marker = Marker()

        marker.type = Marker.CUBE
        marker.scale.x = msg.scale * 0.45
        marker.scale.y = msg.scale * 0.45
        marker.scale.z = msg.scale * 0.45
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0

        return marker

    def processFeedback(self, feedback):
        pose = feedback.pose
        nameid = feedback.marker_name
        nameidx = int(nameid.split("_")[1])
        self.markerserver.applyChanges()
        self.markerposes.poses[nameidx] = pose
        self.markerposes.header.stamp = rospy.Time.now()
        self.pose_pub.publish(self.markerposes)
        
    

if __name__ == "__main__":
    try:
        rospy.init_node("gamma_sources_server")
        rospy.loginfo("Starting Gamma Source Server")
        s = gammaSpawner()
        rospy.spin()
    except rospy.ROSInterruptException: pass