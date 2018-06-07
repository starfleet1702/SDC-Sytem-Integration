#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32;
from scipy.spatial import KDTree
from copy import deepcopy;
import numpy as np

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5;
STOP_BUFFER_WP = 4;
NUM_WP_IN_SIGHT = 20;

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater',log_level=rospy.DEBUG);

        # DONE: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32,self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb);

        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # DONE: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stop_line_wp_idx = None;
        self.cur_vel = None;

        self.last_close_waypoint_idx = None;
        self.last_lane_wp = None;

        self.loop()

    def loop(self):
        #rospy.spin()
        rate = rospy.Rate(50) #50Hz
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                close_waypoint_idx = self.get_close_waypoint_idx()
                self.publish_waypoints(close_waypoint_idx)
            rate.sleep()

    def get_close_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x,y], 1)[1]

        #Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through the closest coordinates
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])

        val = np.dot( cl_vect - prev_vect, pos_vect - cl_vect )

        if val > 0:
            closest_idx = (closest_idx + 1)%len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self, closest_idx):
        lane  = self.gen_lane(closest_idx);
        # rospy.logwarn("Publishing on final waypoint.....");
        # rospy.logwarn("Last wp twist : %s",lane.waypoints[len(lane.waypoints)-1].twist.twist.linear.x)
        self.final_waypoints_pub.publish(lane)


    def gen_lane(self,closest_idx):
        # if(self.last_close_waypoint_idx == closest_idx):
        #     return self.last_lane_wp;

        lane = Lane();
        lane.header = self.base_waypoints.header;

        farthest_idx = closest_idx + LOOKAHEAD_WPS;
        # If Red Light detected
        if self.is_red_light_ahead(closest_idx):
            rospy.logdebug("WaypointUpdater : Found a Red Light Ahead!!");
            # rospy.logwarn("WaypointUpdater : gen_lane : %s", self.stop_line_wp_idx)
            lane.waypoints = self.decelerate_to_stop(closest_idx, farthest_idx);
            # lane.waypoints = self.decelerate_to_stop_dummy(closest_idx, farthest_idx);
        else: # If No Red light Detected
            #rospy.loginfo("WaypointUpdater : Found a Non Red Light Ahead!!");
            #rospy.loginfo("WaypointUpdater : gen_lane : %s",self.stop_line_wp_idx);l
            lane.waypoints = self.base_waypoints.waypoints[closest_idx : farthest_idx];
        # self.last_close_waypoint_idx = closest_idx;
        # self.last_lane_wp = lane;
        return lane;

    def is_red_light_ahead(self,closest_idx):
        return not ((self.stop_line_wp_idx == None) or (self.stop_line_wp_idx == -1) or (self.stop_line_wp_idx > len(self.base_waypoints.waypoints)))  \
               and (self.stop_line_wp_idx - NUM_WP_IN_SIGHT - STOP_BUFFER_WP <= closest_idx  <= self.stop_line_wp_idx + STOP_BUFFER_WP);

    def decelerate_to_stop_dummy(self,closest_idx, farthest_idx):
        waypoints = deepcopy(self.base_waypoints.waypoints[closest_idx:farthest_idx+1]);
        for i,wp in enumerate(waypoints):
            wp.twist.twist.linear.x = 0;
        # rospy.logwarn("Base wp closest_idx twist : %s",self.base_waypoints.waypoints[closest_idx].twist.twist.linear.x);
        return waypoints;

    def decelerate_to_stop(self,closest_idx, farthest_idx):
        # to stop car's front before stop_line
        stop_line_wp_idx_local = self.stop_line_wp_idx - STOP_BUFFER_WP;

        waypoints_before_stop_line = [];
        waypoints_after_stop_line = [];

        # If stop line is in our range of LOOKAHEAD_WPS
        if(farthest_idx >= stop_line_wp_idx_local):
            #last_wp_idx = stop_line_wp_idx_local;
            waypoints_before_stop_line = deepcopy(self.base_waypoints.waypoints[closest_idx : stop_line_wp_idx_local]);
            waypoints_after_stop_line = deepcopy(self.base_waypoints.waypoints[stop_line_wp_idx_local : farthest_idx+1]);
            for i,wp in enumerate(waypoints_after_stop_line):
                wp.twist.twist.linear.x = 0.0;
            total_dist = 0.0;
        else:
            #last_wp_idx = farthest_idx;
            waypoints_before_stop_line = deepcopy(self.base_waypoints.waypoints[closest_idx : farthest_idx + 1]);
            total_dist = self.distance_along_path(self.base_waypoints.waypoints,farthest_idx,stop_line_wp_idx_local);
        rospy.logdebug(" decel : closest_idx: %s ,farthest_idx : %s ,stop_idx : %s ,cur_vel = %s",closest_idx,farthest_idx,stop_line_wp_idx_local,self.cur_vel);
        prev_waypoint_idx = len(waypoints_before_stop_line)-1;

        # Iterating over waypoints_before_stop_line till last_wp in reverse
        for i in range(len(waypoints_before_stop_line)-1,-1,-1):
            # calculating distance for current waypoint from stop_line
            total_dist += self.distance(waypoints_before_stop_line[i].pose.pose.position,waypoints_before_stop_line[prev_waypoint_idx].pose.pose.position);
            vel = math.sqrt(2.0 * MAX_DECEL * total_dist);
            #rospy.logwarn(" decel : vel = %s",vel);
            if vel < 1.0:
                vel = 0.0;
            vel = min(vel,waypoints_before_stop_line[i].twist.twist.linear.x);
            rospy.logdebug(" decel : vel : %s at : %s ",vel,closest_idx+i);
            waypoints_before_stop_line[i].twist.twist.linear.x = vel;
            prev_waypoint_idx = i;
        return waypoints_before_stop_line + waypoints_after_stop_line;


    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
            if( len(self.waypoints_2d) == 0 ):
                rospy.logwarn("Empty Waypoints")
		

    def traffic_cb(self, msg):
        # Done : Callback for /traffic_waypoint message. Implement
        self.stop_line_wp_idx = msg.data;
        # rospy.logwarn("WaypointUpdater : traffic_cb : %s", self.stop_line_wp_idx)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance_along_path(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distance(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z);

    def velocity_cb(self,msg):
        self.cur_vel = msg.twist.linear.x;

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
