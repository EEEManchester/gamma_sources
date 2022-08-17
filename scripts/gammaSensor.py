#! /usr/bin/env python

# General Dependancies
import rospy
from geometry_msgs.msg import TransformStamped, Pose, PoseArray, Quaternion
import tf.transformations as tft
import tf2_ros
import math
from std_srvs.srv import Trigger, TriggerResponse
from radiation_msgs.msg import DoseRate


class gammaSensor():
    def __init__(self):
        # Get parameters
        self.node_name = rospy.get_name()
        self.sensor_frame = rospy.get_param(self.node_name +"/sensor_frame", "gamma_sensor")
        rospy.set_param(self.node_name + "/sensor_frame", self.sensor_frame)
        self.sources_topic = rospy.get_param("~sources_topic", "gamma_sources_server/poses")
        self.source_intensities = rospy.get_param("gamma_sources_server/intensity")
        self.rate = rospy.Rate(1.0) # Hz
        self.global_frame = "map"

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.source_poses = PoseArray()
        self.intensity_msg = DoseRate()

        rospy.Subscriber(self.sources_topic, PoseArray, self.posescb)  # Get positions of gamma sources

        s = rospy.Service(self.node_name + "/update_frame", Trigger, self.service_handler) # Method to trigger a change of TF frame on-the-fly

        self.intensity_publisher = rospy.Publisher(self.node_name + "/intensity", DoseRate, queue_size=1, latch=True)

        # Single TF call
        self.transform = TransformStamped()
        self.calcTF()

        rospy.wait_for_message(self.sources_topic, PoseArray, timeout=5)

        while not rospy.is_shutdown():
            # Calculate TF from sensor frame to world frame
            self.calcTF()
            # Calculate distance to each source
            dists = self.getDistances()
            # Convert distance to intensity
            total_intensity = self.dist2flux(dists)
            # Publish overall intensity
            self.publish(total_intensity)
            self.rate.sleep()

    def service_handler(self, request):
        self.sensor_frame = rospy.get_param("~sensor_frame")
        return TriggerResponse(success = True, message = self.sensor_frame)

    def posescb(self, arraymsg):
        # Update poses of sources in global frame (if moved)
        self.source_poses = arraymsg
        self.global_frame = arraymsg.header.frame_id

    def calcTF(self):
        try:
            # Target Frame, Source Frame, Time, Timeout Duration
            self.transform = self.tfBuffer.lookup_transform(self.sensor_frame, self.global_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.rate.sleep()

    def getDistances(self):
        dists = []
        for pose in self.source_poses.poses:
            v = [self.transform.transform.translation.x + pose.position.x, self.transform.transform.translation.y + pose.position.y, self.transform.transform.translation.z + pose.position.z]
            dist = tft.vector_norm(v)
            dists.append(dist)
        return dists

    def dist2flux(self, distances):
        R = 1E-2
        normalisation = 2/( 1- ( 1 / math.sqrt(1 + R**2) ) )
        # Double check intensities
        self.source_intensities = rospy.get_param("gamma_sources_server/intensity")

        I_Total = 0

        for i, D in enumerate(distances):
            I_D = 0.5 * self.source_intensities[i] * ( 1- ( D / math.sqrt(D**2 + R**2) ) ) * normalisation
            I_Total += I_D

        return I_Total

    def publish(self, intensity_value):
        self.intensity_msg.header.stamp = rospy.Time.now()
        self.intensity_msg.header.frame_id = self.sensor_frame
        self.intensity_msg.radiation_type = 4
        self.intensity_msg.units = 0
        self.intensity_msg.integration_time = 1.0
        self.intensity_msg.rate = intensity_value
        self.intensity_publisher.publish(self.intensity_msg)







if __name__ == "__main__":
    try:
        rospy.init_node("gamma_sensor", anonymous=True)
        rospy.loginfo("Starting Gamma Sensor")
        s = gammaSensor()
        rospy.spin()
    except rospy.ROSInterruptException: pass