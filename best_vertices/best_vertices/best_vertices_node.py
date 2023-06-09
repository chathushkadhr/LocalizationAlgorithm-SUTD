import rclpy
from rclpy.node import Node
import math
from scipy.spatial import distance
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
from tuw_multi_robot_msgs.msg import Graph
from exploration.msg import ExplorationState
from rclpy.qos import QoSProfile
import time

class BestVertices(Node):
    
    
    def __init__(self):
        super().__init__('best_vertices_node')
        self.ver = None
        self.vertices = None
        self.n_robot_pos = None
          
        self.finished_robots = []
        self.map = None
        
        
        self.declare_parameter("segment_topic", "segments")
        self.segment_topic = self.get_parameter("segment_topic").get_parameter_value().string_value      
    
        self.declare_parameter("voronoi_map_topic", "voronoi_map")
        self.voronoi_map_topic = self.get_parameter("voronoi_map_topic").get_parameter_value().string_value  
              
        self.declare_parameter("map_topic", "/robot1/map")
        self.map_topic = self.get_parameter("map_topic").get_parameter_value().string_value      
      
        self.declare_parameter("n_robots", 2)
        self.n_robots = self.get_parameter("n_robots").get_parameter_value().integer_value
        
        self.declare_parameter("human_pos", [0.0,0.0])
        self.human_pos = self.get_parameter("human_pos").get_parameter_value().double_array_value
      

        self.declare_parameter("radius", 2.0)
        self.radius = self.get_parameter("radius").get_parameter_value().double_value

        self.declare_parameter("env", "room_lvl7")
        self.env = self.get_parameter("env").get_parameter_value().string_value

        self.declare_parameter("update_rate_info", 1.0)
        self.update_rate_info = self.get_parameter("update_rate_info").get_parameter_value().double_value                                
        
        
        #Publishers
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array',QoSProfile(depth=10,durability =1))
        self.current_pos_pub = self.create_publisher(MarkerArray, 'current_pos_marker_array',10)
        self.voronoi_map_pub = None
        #Subscribers
        self.subscription = self.create_subscription(Graph,self.segment_topic, self.graph_callback,10)
        self.robot_state_subscription = self.create_subscription(ExplorationState,"/exploration_state", self.robot_state_callback,10)
        self.map_subscription = None
        self.robot_location = [0]*self.n_robots
        self.cur_robot_location = [0]*self.n_robots
        
        self.timer_main = self.create_timer(self.update_rate_info, self.timer_callbaclk)
        self.timer_marker = self.create_timer(0.5, self.PublishCurrentPos)  
        
        self.get_logger().info("n_robots: "+str(self.n_robots))
        self.get_logger().info("human_pos :"+str(self.human_pos))
        self.get_logger().info("radius: "+str(self.radius))
        self.get_logger().info("env: "+str(self.env))
        self.get_logger().info("update_rate_info: "+str(self.update_rate_info))
        
            
    def map_callback(self,msg):
        if self.map == None :
            self.get_logger().info("Complete Map Recieved")
            self.map = msg
        
    
    def graph_callback(self,msg):
        self.vertices = msg.vertices
    
    def robot_state_callback(self,msg):
        self.cur_robot_location[int(msg.robot_id)-1] = [msg.location.position.x,msg.location.position.y]
        if int(msg.status) != int(ExplorationState.DONE):
            return
        if int(msg.robot_id) not in self.finished_robots:
            self.finished_robots.append(int(msg.robot_id))
            self.get_logger().info("Robot"+str(int(msg.robot_id))+" has finished")
            self.robot_location[int(msg.robot_id)-1] = [msg.location.position.x,msg.location.position.y]
            
        if len(self.finished_robots) == self.n_robots and self.map_subscription == None:
            self.get_logger().info("All robots have finished exploration")
            self.map_subscription =  self.create_subscription(OccupancyGrid,self.map_topic, self.map_callback,10)
                

 
    def timer_callbaclk(self): 

        if self.map==None:
            return 
        else:
            if self.voronoi_map_pub ==None:
                self.get_logger().info("Sending to tuw_voronoi_package")
                self.voronoi_map_pub = self.create_publisher(OccupancyGrid,self.voronoi_map_topic,QoSProfile(depth=10,durability =1))
                self.voronoi_map_pub.publish(self.map)

         
         
        if self.vertices == None or len(self.vertices)==0:
            self.get_logger().info("No Graph recieved")
            return
        
        self.get_logger().info("Graph recieved")
        
        V = self.Vertices(self.vertices)
        self.ver = V[:]
        
        self.human_pos = [self.human_pos[0] - self.map.info.origin.position.x , self.human_pos[1]-self.map.info.origin.position.y]
        
        human_pos = tuple(self.human_pos)
        
        try:
            robot_pos = self.RobotPos(self.n_robots, human_pos, self.radius, V) 
        except IndexError as e:
            self.get_logger().info(e.args[0])
            self.timer_main.cancel()
            return
                    
        self.get_logger().info("\nHuman position (red square): {0}".format(human_pos))
        self.get_logger().info("Radius: {0:.2f}".format(self.radius))
        self.get_logger().info("\nRobot positions (green circles): ")   
        markerArray = self.PubMarker(self.human_pos, robot_pos)   
        self.marker_pub.publish(markerArray)
        self.timer_main.cancel()
        
    
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
        return [[round(x, 2),round(y, 2)] for x,y in V]
    
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
                vertices_in_range = [vertices_in_range[j] for j in range(len(vertices_in_range)) if self.Dist(vertices_in_range[j], human_pos) >= radius/3 and self.Dist(vertices_in_range[j], robot_pos[i]) >= radius/3]
            furthest_vertex = self.FurthestVertex(ref_point, vertices_in_range)
            robot_pos += furthest_vertex
        return robot_pos
    
    def RobotPos(self,n_robots, human_pos, radius, V):
        vertices_in_range = self.VerticesInRange(human_pos, radius, V)
        if len(vertices_in_range)==0:
            raise IndexError("No suitable vertex for the human")
        robot_pos = self.FurthestVertex(human_pos, vertices_in_range)
        V.remove(robot_pos[0])
        if n_robots >= 2:
            vertices_in_range = self.VerticesInRange(robot_pos[0], radius, V)
            if len(vertices_in_range)==0:
                raise IndexError("No suitable vertex for a robot")
            vertices_in_range = [vertices_in_range[j] for j in range(len(vertices_in_range)) if self.Dist(vertices_in_range[j], human_pos) >= radius/3 and self.Dist(vertices_in_range[j], robot_pos[0]) >= radius/3]
            robot_pos += self.FurthestVertex(robot_pos[0], vertices_in_range)
        if n_robots >= 3:
            V.remove(robot_pos[1])
            for i in range(2, n_robots):
                robot_pos = self.AddFurthestVertex(robot_pos, human_pos, robot_pos[i-2], robot_pos[i-2], radius, V)
        else:
            pass
        return robot_pos

    def GoalMarker(self, point, markerArray):
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
        marker.pose.position.x = float(point[0]) + self.map.info.origin.position.x
        marker.pose.position.y = float(point[1]) + self.map.info.origin.position.y
        marker.pose.position.z = 0.0
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
        marker.pose.position.x = float(point[0]) + self.map.info.origin.position.x
        marker.pose.position.y = float(point[1]) + self.map.info.origin.position.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        markerArray.markers.append(marker)        
        
        return markerArray
    
    def RobotMarker(self, point, markerArray):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = 3
        marker.id = 0
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.pose.position.x = float(point[0])
        marker.pose.position.y = float(point[1])
        marker.pose.position.z = 0.0
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
            markerArray = self.GoalMarker(max_degs_vertices[i], markerArray)
            
        for loc in self.robot_location:
            markerArray = self.RobotMarker(loc, markerArray)
            
        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1
        return markerArray

    def PublishCurrentPos(self):
        markerArray = MarkerArray()
        for point in self.cur_robot_location:
            if point ==0:
                return
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = 2
            marker.id = 0
            marker.scale.x = 0.6
            marker.scale.y = 0.6
            marker.scale.z = 1.0
            marker.color.r = 0.6
            marker.color.g = 0.5
            marker.color.b = 0.2
            marker.color.a = 1.0
            marker.pose.position.x = float(point[0])
            marker.pose.position.y = float(point[1]) 
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            markerArray.markers.append(marker)
        
        id=0
        for m in markerArray.markers:
            m.id = id
            id += 1    
            
        self.current_pos_pub.publish(markerArray)

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
