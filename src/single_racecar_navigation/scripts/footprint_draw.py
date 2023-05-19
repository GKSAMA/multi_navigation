#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PolygonStamped, PoseStamped, Point
from visualization_msgs.msg import Marker
import tf2_ros
import tf
import math
import copy
import argparse

class FootprintDrawNode:
    namespace = ""
    def __init__(self,ns):
        rospy.Subscriber('/'+ns+'/move_base/local_costmap/footprint',PolygonStamped,self.polygon_callback)
        self.footprint_marker = rospy.Publisher('/footrpint_marker', Marker, queue_size=2)
        
        self.footprint = Marker()
        self.footprint.header.frame_id = '/map'
        self.footprint.header.stamp = rospy.Time.now()
        self.footprint.type = Marker.LINE_LIST
        self.footprint.id = 0
        self.footprint.action = Marker.ADD
        self.footprint.scale.x = 0.07;
        self.footprint.color.a = 1.0;
        self.footprint.color.r = 0.0;
        self.footprint.color.g = 1.0;
        self.footprint.color.b = 0.0;
    
    def polygon_callback(self,msg):
        n = len(msg.polygon.points)
        self.footprint.points = []
        for i in range(n-1):
            p1 = copy.deepcopy(msg.polygon.points[i])
            p2 = copy.deepcopy(msg.polygon.points[i + 1])
            self.footprint.points.append(p1)
            self.footprint.points.append(p2)
        self.footprint.points.append(msg.polygon.points[0])
        self.footprint.points.append(msg.polygon.points[-1])
        self.footprint_marker.publish(self.footprint)

def parser_args(args=None): 
    parser = argparse.ArgumentParser(description='namespace')
    parser.add_argument('--namespace', metavar='ns', default='/smart_0')
    return parser.parse_args(args=args)

if __name__ == '__main__':
    args=parser_args(rospy.myargv()[1:])
    rospy.init_node("footprint_draw_node")
    node = FootprintDrawNode(args.namespace)
    rospy.spin()