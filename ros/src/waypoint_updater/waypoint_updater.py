#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import time
import math
import copy

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

LOOKAHEAD_WPS = 40 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # not yet implemented, so leaving commented out
        #rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        # holds the current vehicle pose from /current_pose topic
        self.current_pose = None

        # holds list of waypoints that will be loaded over /base_waypoints topic
        self.base_waypoints = None

        self.traffic_light_state = False

        rospy.spin()

    def pose_cb(self, msg):
        self.current_pose = msg.pose
        print("current pose set!")
        if not self.traffic_light_state:
            self.publish_waypoints()

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints.waypoints
        #rospy.logwarn([w.twist.twist.linear.x for w in self.base_waypoints])
        # print("waypoints set!")

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        waypoint = msg.data

        if waypoint==-1:
            rospy.logwarn('no light')
            self.traffic_light_state = False
            return

        LIGHT_MAX_DIST = 42

        # ignore red light if it's too far ahead
        nearest_waypoint = self.nearest_waypoint()
        if waypoint-nearest_waypoint > LIGHT_MAX_DIST:
            rospy.logwarn('light, too far: %d', waypoint-nearest_waypoint)
            return

        self.traffic_light_state = True

        new_waypoints = []
        waypoint -= 7      # adjustment to stop before the line
        # rospy.logwarn('traffic_cb dest_waypoint: %d, current_waypoint: %d', waypoint, self.nearest_waypoint()) #
        waypoints_to_stop = waypoint - nearest_waypoint
        rospy.logwarn('nearest_waypoint: %d; waypoints_to_stop; %d; waypoint: %d', nearest_waypoint, waypoints_to_stop, waypoint)

        for i in range(waypoints_to_stop):
            # rospy.logwarn('updating velocity for waypoint %d', i)
            p = 1. - float(i + 1) / float(waypoints_to_stop)
            #rospy.logwarn('p: %.3f', p)
            cur_v = self.get_waypoint_velocity(self.base_waypoints[i])
            new_v = cur_v * p
            # rospy.logwarn('   cur_v: %.3f, new_v: %.3f', cur_v, new_v)
            waypoint_obj = copy.deepcopy(self.base_waypoints[nearest_waypoint + i])
            waypoint_obj.twist.twist.linear.x = new_v
            new_waypoints.append(waypoint_obj)

        # add extra waypoints with 0 speed after the stopping light
        for i in range(LOOKAHEAD_WPS):
            waypoint_obj = copy.deepcopy(self.base_waypoints[waypoint+i])
            waypoint_obj.twist.twist.linear.x = 0
            new_waypoints.append(waypoint_obj)

        rospy.logwarn('light')
        self.publish_waypoints(waypoints=new_waypoints)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def position_distance(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

    def position_waypoint_distance(self, a, wp):
        return math.sqrt((a.x-wp.pose.pose.position.x)**2 + (a.y-wp.pose.pose.position.y)**2)  #+ (a.z-b.z)**2)

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        for i in range(wp1, wp2+1):
            dist += position_distance(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def closest_waypoint_search(self, cur_wp, waypoints, begin, end):
        if end - begin <= 3:
            nearest_waypoint_index = -1
            nearest_distance = 1e400
            for i in range(begin,end+1):
                dist = self.position_waypoint_distance(cur_wp, waypoints[i])
                if dist<nearest_distance:
                    nearest_distance = dist
                    nearest_waypoint_index = i
            return nearest_waypoint_index, nearest_distance
        low = int((begin * 2 + end) / 3)
        high = int((begin + end * 2) / 3)
        dist_low = self.position_waypoint_distance(cur_wp, waypoints[low])
        dist_high = self.position_waypoint_distance(cur_wp, waypoints[high])
        if dist_low < dist_high:
            return self.closest_waypoint_search(cur_wp, waypoints, begin, high - 1)
        else:
            return self.closest_waypoint_search(cur_wp, waypoints, low + 1, end)

    def nearest_waypoint(self):
        nearest_waypoint_index, nearest_distance = self.closest_waypoint_search(self.current_pose.position,
                                                                                self.base_waypoints,
                                                                                0,
                                                                                len(self.base_waypoints)-1)
        # rospy.logwarn('nearest wp: %d', nearest_waypoint_index)
        # rospy.logwarn('nearest wp dist: %.3f', nearest_distance)
        # rospy.logwarn('nearest search: %.3f'%(1/(time.time()-s)))

        return nearest_waypoint_index

    def publish_waypoints(self, waypoints=None):
        if self.base_waypoints is None:
            return

        if waypoints is None:
            nearest_waypoint_index = self.nearest_waypoint()
            closing_waypoint_index = nearest_waypoint_index + LOOKAHEAD_WPS

            waypoints = self.base_waypoints[nearest_waypoint_index:closing_waypoint_index]
            #print("publishing {} waypoints".format(len(waypoints_to_publish)))

        #rospy.logwarn([w.twist.twist.linear.x for w in waypoints])
        self.final_waypoints_pub.publish(Lane(waypoints=waypoints))

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
