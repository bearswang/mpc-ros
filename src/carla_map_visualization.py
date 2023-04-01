#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import numpy as np
import carla


import rospy
from rospy import ROSException

from std_msgs.msg import String
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from carla_msgs.msg import CarlaWorldInfo

COLOR_ALUMINIUM_2 = ColorRGBA(186.0 / 255.0, 189.0 / 255.0, 182.0 / 255.0, 1)
COLOR_SKY_BLUE_0 = ColorRGBA(114.0 / 255.0, 159.0 / 255.0, 207.0 / 255.0, 1)
COLOR_CHAMELEON_0 = ColorRGBA(138.0 / 255.0, 226.0 / 255.0, 52.0 / 255.0, 1)
COLOR_SCARLET_RED_0 = ColorRGBA(239.0 / 255.0, 41.0 / 255.0, 41.0 / 255.0, 1)
COLOR_ORANGE_0 = ColorRGBA(252.0 / 255.0, 175.0 / 255.0, 62.0 / 255.0, 1)


class CarlaMapVisualization:
    """
    Pseudo opendrive sensor
    """

    def __init__(self):
        self.world = None
        self.connect_to_carla()
        self.map = self.world.get_map()
        self.map_name = self.map.name
        rospy.loginfo("Map Visualization Node: Loading {} map!".format(self.map_name))
        self.map_viz_publisher = rospy.Publisher('/carla/map_visualization', MarkerArray, latch=True, queue_size=1)

        self.id = 0
        self.marker_array = MarkerArray()

    def connect_to_carla(self):

        rospy.loginfo("Map Visualization Node: Waiting for CARLA world (topic: /carla/world_info)...")
        try:
            rospy.wait_for_message("/carla/world_info", CarlaWorldInfo, timeout=rospy.Duration(secs=15))
        except ROSException as e:
            rospy.logerr("Map Visualization Node: Error while waiting for world info: {}".format(e))
            raise e

        host = rospy.get_param("~"+"host", "127.0.0.1")
        port = rospy.get_param("~"+"port", 2000)
        timeout = rospy.get_param("timeout", 10)
        rospy.loginfo("Map Visualization Node: CARLA world available. Trying to connect to {host}:{port}".format(
            host=host, port=port))

        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(timeout)

        try:
            self.world = carla_client.get_world()
        except RuntimeError as e:
            rospy.logerr("Error while connecting to Carla: {}".format(e))
            raise e

        rospy.loginfo("Connected to Carla.")

    def publish_msgs(self):
        """
        Function (override) to update this object.
        """
        self.draw_map()

        rospy.loginfo(
            "Map Visualization Node: Got {} markers for carla map visualization".format(len(self.marker_array.markers)))

        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.map_viz_publisher.publish(self.marker_array)
            r.sleep()

    @staticmethod
    def lateral_shift(transform, shift):
        """Makes a lateral shift of the forward vector of a transform"""
        transform.rotation.yaw += 90
        return transform.location + shift * transform.get_forward_vector()

    def set_marker_id(self):
        self.id += 1
        return self.id - 1

    def add_arrow_line_marker(self, transform):
        arrow_marker = Marker()
        arrow_marker.type = Marker.LINE_LIST
        arrow_marker.header.frame_id = "map"
        arrow_marker.id = self.set_marker_id()
        arrow_marker.ns = "map_visulization"
        arrow_marker.color = ColorRGBA(0.8, 0.8, 0.8, 1)
        arrow_marker.scale.x = 0.2
        arrow_marker.pose.orientation.w = 1
        transform.rotation.yaw += 180
        forward = transform.get_forward_vector()
        transform.rotation.yaw += 90
        right_dir = transform.get_forward_vector()
        end = transform.location
        start = end - 2.0 * forward
        right = start + 0.8 * forward + 0.4 * right_dir
        left = start + 0.8 * forward - 0.4 * right_dir
        points = [start, end, start, left, start, right]
        for p in points:
            point = Point()
            point.x = p.x
            point.y = -p.y
            # For 3D visualization
            # point.z = p.z
            # For 2D visualization
            point.z = -2
            arrow_marker.points.append(point)
        self.marker_array.markers.append(arrow_marker)

    def add_line_strip_marker(self, color=None, points=None):
        marker = Marker()
        marker.id = self.set_marker_id()
        marker.type = Marker.LINE_STRIP
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.ns = "map_visulization"

        if color is None:
            marker.color = ColorRGBA(1, 1, 1, 1)
        else:
            marker.color = color

        marker.scale.x = 0.25
        marker.pose.orientation.w = 1

        if points is not None:
            for p in points:
                point = Point()
                point.x = p.x
                point.y = -p.y
                point.z = -2
                # point.z = 0
                marker.points.append(point)
        self.marker_array.markers.append(marker)
        return marker

    def draw_map(self):
        precision = 0.1
        topology = self.map.get_topology()
        topology = [x[0] for x in topology]
        topology = sorted(topology, key=lambda w: w.transform.location.z)
        set_waypoints = []
        for waypoint in topology:
            waypoints = [waypoint]
            nxt = waypoint.next(precision)
            if len(nxt) > 0:
                nxt = nxt[0]
                while nxt.road_id == waypoint.road_id:
                    waypoints.append(nxt)
                    nxt = nxt.next(precision)
                    if len(nxt) > 0:
                        nxt = nxt[0]
                    else:
                        break
            set_waypoints.append(waypoints)

        for waypoints in set_waypoints:
            waypoint = waypoints[0]
            road_left_side = [self.lateral_shift(w.transform, -w.lane_width * 0.5) for w in waypoints]
            road_right_side = [self.lateral_shift(w.transform, w.lane_width * 0.5) for w in waypoints]
            # road_points = road_left_side + [x for x in reversed(road_right_side)]
            # self.add_line_strip_marker(points=road_points)

            if len(road_left_side) > 2:
                self.add_line_strip_marker(points=road_left_side)
            if len(road_right_side) > 2:
                self.add_line_strip_marker(points=road_right_side)

            if not waypoint.is_junction:
                for n, wp in enumerate(waypoints):
                    if ((n + 1) % 400) == 0:
                        self.add_arrow_line_marker(wp.transform)


def main(args=None):
    """
    main function
    """
    rospy.init_node("carla_map_visualization", args)

    carla_map_visualization = None
    try:
        carla_map_visualization = CarlaMapVisualization()
        carla_map_visualization.publish_msgs()

        rospy.spin()
    except (RuntimeError, ROSException):
        pass
    except KeyboardInterrupt:
        print("User requested shut down.")
    finally:
        print("Shutting down.")


if __name__ == "__main__":
    main()