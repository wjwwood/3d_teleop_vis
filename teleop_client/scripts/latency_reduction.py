#!/usr/bin/env python

"""
This is a ROS node that takes the cmd_vel messages from the teleoperation
and provides predictions of the vehicle position which can be visualized
in rviz.

Additionally, this node serves as a relay, which relays command velocities to
the robot computer and telemtry to the client computer's visualization system.

By passing through this node, artificial latency can be introduced and 
statistics can be gathered.
"""

import sys
from time import sleep
from threading import Lock, Thread
from copy import deepcopy
from Queue import Queue, Empty

import roslib; roslib.load_manifest('teleop_client')
import rospy

from tf.transformations import euler_from_quaternion as efq
from tf.transformations import quaternion_from_euler as qfe
from tf.msg import tfMessage

from dynamic_reconfigure.server import Server
from teleop_client.cfg import LatencyReductionConfig

from geometry_msgs.msg import Twist, PoseStamped, PoseArray, Pose
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import LaserScan

from teleop_client.differential_kinematics import DifferentialKinematics

import shapely.geometry as geo

class LatentPublisher(object):
    """LatentPublisher"""
    def __init__(self, topic_name, topic_type, queue_size=0):
        self.pub = rospy.Publisher(topic_name, topic_type)
        self.queue = Queue(queue_size)
        self.thread = Thread(target=self.run)
        self.thread.start()
    
    def run(self):
        while not rospy.is_shutdown():
            try:
                msg, time = self.queue.get(True, 0.1)
                now = rospy.Time.now().to_sec()
                diff = time - now
                if diff > 0.001:
                    sleep(diff)
                self.pub.publish(msg)
            except Empty as e:
                pass

    def publish(self, msg, time):
        self.queue.put((msg, time))

class LatencyReductionNode(object):
    """Latency Reduction Node"""
    def __init__(self):
        rospy.init_node('latency_reduction')

        self.cmd_vel_list = []
        self.cmd_vel_lock = Lock()
        self.current_pose = None
        self.last_marker_length = 0

        self.get_ros_params()

        self.setup_ros_comms()

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.step_vehicle_modeling(self.cmd_vel_list,
                                       deepcopy(self.current_pose))
            rate.sleep()

    def prune_cmd_vels(self, cmd_vel_list, stamp):
        """Takse a list of cmd_vel's and prunes them using a time stamp"""
        time = stamp.to_sec()
        for index, cmd_vel in enumerate(cmd_vel_list):
            if cmd_vel[0] <= time:
                del cmd_vel_list[index]

    def simulate_vehicle(self, current_pose, cmd_vel_list, period=0.05):
        """
        Takes the current position, a list of user inputs, and a step size
        and simulates the vehicle moving forward in time.
        """
        now = rospy.Time.now().to_sec()
        time = current_pose.header.stamp.to_sec()
        current_cmd_vel = cmd_vel_list.pop(0)
        sim_path = []
        state = [current_pose.pose.pose.position.x,
                 current_pose.pose.pose.position.y,
                 efq([current_pose.pose.pose.orientation.x,
                      current_pose.pose.pose.orientation.y,
                      current_pose.pose.pose.orientation.z,
                      current_pose.pose.pose.orientation.w])]
        dk = DifferentialKinematics(1.0, 1.0, state[0], state[1], state[2][2])
        # print('Entering sim...')
        while time < now and not rospy.is_shutdown():
            # print(time, now, now - time)
            # Increase time
            time += period
            # Check for new input
            if cmd_vel_list and current_cmd_vel[0] < time:
                if cmd_vel_list[0][0] <= time:
                    current_cmd_vel = cmd_vel_list.pop(0)
            # Use the input, the state, and the period to step the model
            dk.linear_velocity = current_cmd_vel[1].linear.x
            dk.angular_velocity = current_cmd_vel[1].angular.z
            x, y, w = dk.step_simulation(period)
            sim_path.append([time, x, y, w])
        # print('Exiting sim...')
        return sim_path

    def step_vehicle_modeling(self, cmd_vel_list, current_pose):
        """
        This function takes the current cmd_vel's and predicts
        the vehicle position since the last position update.
        """
        # If there is no inital pose, continue without predictions
        if current_pose == None:
            return
        my_cmd_vel_list = None
        # Duplicate the list
        with self.cmd_vel_lock:
            # Prune out of date cmd_vels
            self.prune_cmd_vels(cmd_vel_list, current_pose.header.stamp)
            # Duplicate the list
            my_cmd_vel_list = list(cmd_vel_list)
        # Use the current pose and cmd_vels to step forward
        if my_cmd_vel_list != []:
            sim_path = self.simulate_vehicle(current_pose, my_cmd_vel_list)
        else:
            sim_path = None
        # Publish the results
        # self.publish_sim_path_as_path(sim_path, current_pose.header.frame_id)
        self.publish_sim_path_as_markers(sim_path, current_pose.header.frame_id)
        # self.publish_sim_path_as_pose_array(sim_path, current_pose.header.frame_id)

    def publish_sim_path_as_path(self, sim_path, frame_id):
        """Publishes the given sim path as a ROS Path msg"""
        msg = Path()
        msg.header.frame_id = frame_id
        msg.header.stamp = rospy.Time.now()
        if sim_path:
            for waypoint in sim_path:
                pose = PoseStamped()
                pose.pose.position.x = waypoint[1]
                pose.pose.position.y = waypoint[2]
                quat = qfe(0, 0, waypoint[3])
                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]
                msg.poses.append(pose)
        self.path_pub.publish(msg)

    def publish_sim_path_as_pose_array(self, sim_path, frame_id):
        """Publishes the given sim path as a ROS pose array"""
        msg = PoseArray()
        msg.header.frame_id = frame_id
        msg.header.stamp = rospy.Time.now()
        if sim_path:
            for waypoint in sim_path:
                pose = Pose()
                pose.position.x = waypoint[1]
                pose.position.y = waypoint[2]
                quat = qfe(0, 0, waypoint[3])
                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]
                msg.poses.append(pose)
        self.pose_array_pub.publish(msg)

    def publish_sim_path_as_markers(self, sim_path, frame_id):
        """Publishes the given sim apth as a set of ROS markers"""
        msg = MarkerArray()
        now = rospy.Time.now()
        if not sim_path or len(sim_path) < self.last_marker_length:
            diff = self.last_marker_length
            offset = 0
            if sim_path:
                offset = len(sim_path)
                diff -= offset
            for index in range(diff+1):
                marker = Marker()
                marker.header.frame_id = frame_id
                marker.ns = 'sim_path'
                marker.id = offset + index - 1
                marker.action = Marker.DELETE
        if sim_path:
            for index, waypoint in enumerate(sim_path):
                marker = Marker()
                marker.header.frame_id = frame_id
                marker.ns = 'sim_path'
                marker.id = index
                marker.type = Marker.ARROW
                marker.action = Marker.MODIFY
                marker.pose.position.x = waypoint[1]
                marker.pose.position.y = waypoint[2]
                quat = qfe(0, 0, waypoint[3])
                marker.pose.orientation.x = quat[0]
                marker.pose.orientation.y = quat[1]
                marker.pose.orientation.z = quat[2]
                marker.pose.orientation.w = quat[3]
                marker.scale.x = 0.25
                marker.scale.y = 0.25
                marker.scale.z = 0.25
                marker.color.r = 0.25
                marker.color.g = 0.25
                marker.color.b = 0.25
                marker.color.a = 0.25
                msg.markers.append(marker)
            waypoint = sim_path[-1]
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.ns = 'prediction'
            marker.id = 0
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.MODIFY
            marker.pose.position.x = waypoint[1]
            marker.pose.position.y = waypoint[2]
            marker.pose.position.z = 0.13
            quat = qfe(0, 0, waypoint[3])
            marker.pose.orientation.x = quat[0]
            marker.pose.orientation.y = quat[1]
            marker.pose.orientation.z = quat[2]
            marker.pose.orientation.w = quat[3]
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1
            marker.color.r = 1
            marker.color.g = 0
            marker.color.b = 0
            marker.color.a = 1
            marker.mesh_resource = 'package://gavlab_atrv_description/meshes/atrv_chassis.dae'
            marker.mesh_use_embedded_materials = True
            msg.markers.append(marker)
        self.marker_array_pub.publish(msg)

    def on_cmd_vel(self, msg):
        """This is the callback to handle incoming cmd_vel's"""
        # now = rospy.Time.now().to_sec()
        # time = now + self.send_latency
        # with self.cmd_vel_lock:
        #     self.cmd_vel_list.append((time, msg))
        # diff = time - now
        # if diff > 0.001:
        #     sleep(diff)
        # self.cmd_vel_pub.publish(msg)
        time = rospy.Time.now().to_sec() + self.send_latency
        self.cmd_vel_pub.publish(msg, time)
        with self.cmd_vel_lock:
            self.cmd_vel_list.append((time, msg))

    def on_odom(self, msg):
        """Called when new vehicle telemtry is received"""
        now = rospy.Time.now().to_sec()
        time = msg.header.stamp.to_sec()
        if self.use_one_latency:
            time += self.send_latency
        else:
            time += self.receive_latency
        diff = time - now
        if diff > 0:
            sleep(diff)
        self.current_pose = msg

    def on_tf(self, msg):
        """Used to delay tf broadcast"""
        now = rospy.Time.now().to_sec()
        time = msg.transforms[0].header.stamp.to_sec()
        if self.use_one_latency:
            time += self.send_latency
        else:
            time += self.receive_latency
        diff = time - now
        if diff > 0:
            sleep(diff)
        self.tf_pub.publish(msg)

    def on_dynamic_reconfigure(self, config, level):
        """This is the callback for dynamic reconfigure events"""
        self.send_latency = config['send_latency']
        self.receive_latency = config['receive_latency']
        self.use_one_latency = config['use_one_latency']
        return config

    def on_scan(self, msg):
        now = rospy.Time.now().to_sec()
        time = msg.header.stamp.to_sec()
        if self.use_one_latency:
            time += self.send_latency
        else:
            time += self.receive_latency
        diff = time - now
        if diff > 0:
            sleep(diff)
        self.scan_pub.publish(msg)

    def setup_ros_comms(self):
        """Sets up ROS communications"""
        self.dr_server = Server(LatencyReductionConfig, self.on_dynamic_reconfigure)
        self.cmd_vel_pub = LatentPublisher('/atrv_node/cmd_vel', Twist)
        # self.cmd_vel_pub = rospy.Publisher('/atrv_node/cmd_vel', Twist)
        # self.cmd_vel_pub = rospy.Publisher('/latency_reduction/cmd_vel', Twist)
        self.path_pub = rospy.Publisher('/model_path', Path)
        self.pose_array_pub = rospy.Publisher('/path_poses', PoseArray)
        self.marker_array_pub = rospy.Publisher('/path_markers', MarkerArray)
        self.tf_pub = rospy.Publisher('/tf_delayed', tfMessage)
        self.scan_pub = rospy.Publisher('/scan_delayed', LaserScan)
        from time import sleep
        sleep(1.0)
        rospy.Subscriber('/cmd_vel', Twist, self.on_cmd_vel, queue_size=1000)
        # rospy.Subscriber('/atrv_node/cmd_vel', Twist, self.on_cmd_vel)
        rospy.Subscriber('/atrv_node/odom', Odometry, self.on_odom)
        rospy.Subscriber('/tf', tfMessage, self.on_tf)
        rospy.Subscriber('/scan_filtered', LaserScan, self.on_scan)

    def get_ros_params(self):
        """Gets the ROS parameters"""
        self.send_latency = rospy.get_param('send_latency', 0.0)
        self.receive_latency = rospy.get_param('receive_latency', 0.0)
        self.use_one_latency = rospy.get_param('use_one_latency', False)

if __name__ == '__main__':
    lrn = LatencyReductionNode()
