import rclpy
from rclpy.node import Node
import math
from scipy.spatial import distance
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from tuw_multi_robot_msgs.msg import Graph




class BestVertices(Node):
    
    
    def __init__(self):
        super().__init__('best_vertices_node')
        
        self.msg_recived=0 
        self.vertices = None
        
        self.subscription = self.create_subscription(Graph,"/segments", self.callback,10)
    
        self.declare_parameter("n_robots", 6)
        self.n_robots = self.get_parameter("n_robots").get_parameter_value().integer_value
        
        self.declare_parameter("human_pos", [6,28])
        self.human_pos = self.get_parameter("human_pos").get_parameter_value().integer_array_value
        

        self.declare_parameter("radius", 12)
        self.n_robots = self.get_parameter("n_robots").get_parameter_value().integer_value
        self.radius = self.get_parameter("radius").get_parameter_value().integer_value

        self.declare_parameter("env", "room_lvl7")
        self.env = self.get_parameter("env").get_parameter_value().string_value

        self.declare_parameter("update_rate_infoespace", 1.0)
        self.update_rate_infoespace = self.get_parameter("update_rate_infoespace").get_parameter_value().double_value                                
        
        
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array',10)
        self.markerArray = self.PubMarker(self.human_pos, self.robot_pos)
        
        self.timer_main = self.create_timer(2, self.pub_marker)

        
        self.get_logger().info("n_robots: "+str(self.n_robots))
        self.get_logger().info("human_pos :"+str(self.human_pos))
        self.get_logger().info("radius: "+str(self.radius))
        self.get_logger().info("env: "+str(self.env))
        self.get_logger().info("update_rate_infoespace: "+str(self.update_rate_infoespace))
    
    def callback(self,msg):
        if not self.msg_recived:
            self.vertices = msg.vertices
            self.msg_recived=1
            
        
    def pub_marker(self):
        self.marker_pub.publish(self.markerArray)
        self.get_logger().info("Hello World")
    
    def Dist(self,a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def Vertices(self, vertices):
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
    
    def VerticesInRange(self,pos, radius, V):
        return [V[i] for i in range(len(V)) if self.Dist(V[i], pos) < radius]   
    
    def FurthestVertex(self,point, V):
        furthest_ind = distance.cdist([point], V).argmax()
        return [V[furthest_ind]]
    
    def AddFurthestVertex(self,robot_pos, human_pos, ref_point, point_to_remove, radius, V):
        if point_to_remove in V:
            V.remove(point_to_remove)
        vertices_in_range = self.VerticesInRange(ref_point, radius, V)
        if not vertices_in_range:
            print("\nThere is no vertex in a radius of {0:.2f} from the vertex {1}.".format(radius, ref_point))
        else:
            for i in range(len(robot_pos)):
                vertices_in_range = [vertices_in_range[j] for j in range(len(vertices_in_range)) if Dist(vertices_in_range[j], human_pos) >= radius/3 and Dist(vertices_in_range[j], robot_pos[i]) >= radius/3]
            furthest_vertex = self.FurthestVertex(ref_point, vertices_in_range)
            robot_pos += furthest_vertex
        return robot_pos
    
    def RobotPos(self,n_robots, human_pos, radius, V):
        vertices_in_range = self.VerticesInRange(human_pos, radius, V)
        robot_pos = self.FurthestVertex(human_pos, vertices_in_range)
        V.remove(robot_pos[0])
        if n_robots >= 2:
            vertices_in_range = self.VerticesInRange(robot_pos[0], radius, V)
            vertices_in_range = [vertices_in_range[j] for j in range(len(vertices_in_range)) if Dist(vertices_in_range[j], human_pos) >= radius/3 and Dist(vertices_in_range[j], robot_pos[0]) >= radius/3]
            robot_pos += self.FurthestVertex(robot_pos[0], vertices_in_range)
        if n_robots >= 3:
            V.remove(robot_pos[1])
            for i in range(2, n_robots):
                robot_pos = self.AddFurthestVertex(robot_pos, human_pos, robot_pos[i-2], robot_pos[i-2], radius, V)
        else:
            pass
        return robot_pos

    def RobotMarker(self, point, markerArray):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = self.get_clock().now().to_msg()
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

    def HumanMarker(self, point, markerArray):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = self.get_clock().now().to_msg()
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

    def PubMarker(self,human_pos, max_degs_vertices):
        markerArray = MarkerArray()
        markerArray = self.HumanMarker(human_pos, markerArray)
        for i in range(len(max_degs_vertices)):
            markerArray = self.RobotMarker(max_degs_vertices[i], markerArray)
        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1
        return markerArray



def main(args=None):
    rclpy.init(args=args)

    node = BestVertices()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except :
        print("Error")   
