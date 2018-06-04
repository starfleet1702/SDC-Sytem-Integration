#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree;

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # -------------- Subscribers -----------------------------
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # DONE: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint',Waypoint,self.traffic_cb);
        rospy.Subscriber('/obstacle_waypoint',Waypoint,self.obstacle_cb);

        # -------------- Subscribers -----------------------------
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.car_pose = None;
        self.base_waypoints = None;
        self.waypoints_2d = None;
        
        
        #self.waypoints_to_publish = None;
        self.loop();

        # spin() simply keeps python from exiting until this node is stopped
        # rospy.spin()

    def loop():
        rate = rospy.Rate(50) # in Hz
        waypoints = [];
        while not rospy.is_shutdowm():
            if self.pose and self,base_waypoints:
                closest_wp_idx = self.find_closest_waypoint();
                self.publish_waypoints(closest_wp_idx);
            rate.sleep();
    
    def find_closest_waypoint(self):
        x = self.car_pose.pose.position.x;
        y = self.car_pose.pose.position.y;
        # KDTree.query(x, k=1, eps=0, p=2, distance_upper_bound=inf)
        # k : integer : The number of nearest neighbors to return
        # returns [[distances][indices]] 
        # return tuple (dist,idx) if only 1 point is passed to query
        closest_wp_idx = self.waypoint_tree.query([x,y], 1)[1];

        # checking if the closest_wp is in front of the car or in the back
        closest_wp = self.waypoints_2d[closest_wp_idx];
        prev_wp = self.waypoints_2d[closest_wp_idx-1];

        # Equations for hyper plane through closest_wp
        closest_wp_vect = np.array(closest_wp);
        cur_pose_vect = np.array([x,y]);
        prev_wp_vect = np.array(prev_wp);

        dot_prod = np.dot(closest_wp_vect - prev_wp_vect, cur_pose_vect - closest_wp_vect);
        if dot_prod>0.0:
            # take next waypoint
            # DOUBT : Why are we using modulo , are we considering full path as a circular path?
            closest_wp_idx = (closest_wp_idx+1) % len(self.waypoints_2d);
        return closest_wp_idx;

    def publish_waypoints(self,closest_wp_idx):
        # DOUBT : What's Lane and how n where to use it?
        lane = Lane();
        lane.header = self.base_waypoints.header;
        lane.waypoints = self.base_waypoints[ closest_wp_idx : closest_wp_idx + LOOKAHEAD_WPS ]
        self.final_waypoints_pub.publish(lane);

    def pose_cb(self, msg):
        # TODO: Implement
        self.car_pose = msg.PoseStamped;

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # DOUBT
        # Storing the base waypoints
        self.base_waypoints = waypoints;
        if not self.waypoints_2d:
            self.waypoints_2d = [[wp.pose.pose.position.x , wp.pose.pose.position.y] for wp in waypoints.waypoint];
            self.waypoint_tree = KDTree(self.waypoints_2d);
            
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
