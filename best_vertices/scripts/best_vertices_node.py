#! /usr/bin/env python
import rospy
import math
from scipy.spatial import distance
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from tuw_multi_robot_msgs.msg import Graph

def Dist(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def Vertices(vertices):
    V = set()
    for i in range(len(vertices)):
        seg = vertices[i]
        p1 = seg.path[0]
        p2 = seg.path[-1]
        start_point = (p1.x, p1.y)
        end_point = (p2.x, p2.y)
        V.add(start_point)
        V.add(end_point)
    V = list(V)
    return V

def VerticesInRange(pos, radius, V):
    return [V[i] for i in range(len(V)) if Dist(V[i], pos) < radius]

def FurthestVertex(point, V):
    furthest_ind = distance.cdist([point], V).argmax()
    return [V[furthest_ind]]

def AddFurthestVertex(robot_pos, human_pos, ref_point, point_to_remove, radius, V):
    if point_to_remove in V:
        V.remove(point_to_remove)
    vertices_in_range = VerticesInRange(ref_point, radius, V)
    if not vertices_in_range:
        print("\nThere is no vertex in a radius of {0:.2f} from the vertex {1}.".format(radius, ref_point))
    else:
        for i in range(len(robot_pos)):
            vertices_in_range = [vertices_in_range[j] for j in range(len(vertices_in_range)) if Dist(vertices_in_range[j], human_pos) >= radius/3 and Dist(vertices_in_range[j], robot_pos[i]) >= radius/3]
        furthest_vertex = FurthestVertex(ref_point, vertices_in_range)
        robot_pos += furthest_vertex
    return robot_pos

def RobotPos(n_robots, human_pos, radius, V):
    vertices_in_range = VerticesInRange(human_pos, radius, V)
    robot_pos = FurthestVertex(human_pos, vertices_in_range)
    V.remove(robot_pos[0])
    if n_robots >= 2:
        vertices_in_range = VerticesInRange(robot_pos[0], radius, V)
        vertices_in_range = [vertices_in_range[j] for j in range(len(vertices_in_range)) if Dist(vertices_in_range[j], human_pos) >= radius/3 and Dist(vertices_in_range[j], robot_pos[0]) >= radius/3]
        robot_pos += FurthestVertex(robot_pos[0], vertices_in_range)
    if n_robots >= 3:
        V.remove(robot_pos[1])
        for i in range(2, n_robots):
            robot_pos = AddFurthestVertex(robot_pos, human_pos, robot_pos[i-2], robot_pos[i-2], radius, V)
    else:
        pass
    return robot_pos

def RobotMarker(point, markerArray):
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.header.stamp = rospy.Time.now()
    marker.type = 2
    marker.id = 0
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.pose.position.x = point[0]
    marker.pose.position.y = point[1]
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    markerArray.markers.append(marker)
    return markerArray

def HumanMarker(point, markerArray):
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.header.stamp = rospy.Time.now()
    marker.type = 1
    marker.id = 0
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.pose.position.x = point[0]
    marker.pose.position.y = point[1]
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    markerArray.markers.append(marker)
    return markerArray

def PubMarker(human_pos, max_degs_vertices):
    markerArray = MarkerArray()
    markerArray = HumanMarker(human_pos, markerArray)
    for i in range(len(max_degs_vertices)):
        markerArray = RobotMarker(max_degs_vertices[i], markerArray)
    id = 0
    for m in markerArray.markers:
        m.id = id
        id += 1
    return markerArray

def main():
    rospy.init_node('best_vertices_node', anonymous=True)
    msg = rospy.wait_for_message("/segments", Graph)
    vertices = msg.vertices
    env = rospy.get_param("/best_vertices_node/env")
    n_robots = rospy.get_param("/best_vertices_node/n_robots")
    human_pos = rospy.get_param("/best_vertices_node/human_pos")
    radius = rospy.get_param("/best_vertices_node/radius")
    
    V = Vertices(vertices)
    human_pos = tuple(human_pos)
    robot_pos = RobotPos(n_robots, human_pos, radius, V)

    print("\nHuman position (red square): {0}".format(human_pos))
    print("Radius: {0:.2f}".format(radius))
    print("\nRobot positions (green circles): ")
    print(robot_pos)

    marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
    markerArray = PubMarker(human_pos, robot_pos)
    while not rospy.is_shutdown():
        marker_pub.publish(markerArray)
        rospy.rostime.wallsleep(1.0)

    rospy.spin()
  
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass