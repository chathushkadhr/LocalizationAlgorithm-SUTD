import rclpy
from rclpy.node import Node
import math
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile

import numpy as np
from scipy.optimize import linear_sum_assignment

class RobotsToGoals(Node):
    
    
    def __init__(self):
        super().__init__('robots_to_goals_node')
        
        self.HEIGHT = 0
        self.WIDTH = 0
        self.originx = 0
        self.originy = 0
        self.resolution = 0
        self.map = None
        self.id = -1
       
        self.subscription = self.create_subscription(OccupancyGrid,"/map", self.map_callback,QoSProfile(depth=10,durability =1))
       
        self.rob_pos = [[2,3],
                        [7,10],
                        [25,30],
                        [6.2,16],
                        [5,10],
                        [10,28]]
        
        self.goal_pos = [[3.1000000461935997,16.400000244379044],
                         [5.650000084191561,5.200000077486038],
                         [10.900000162422657,25.00000037252903],
                         [21.950000327080488,25.100000374019146],
                         [2.600000038743019,12.10000018030405],
                         [13.90000020712614,29.000000432133675]]
        
               
        # self.rob_pos = [[1.4,2.8],
        #                 [2.1,13.2],
        #                 [5.2,13.7],
        #                 [4.3,25.5],
        #                 [16.9,25.8],]
        
        # self.goal_pos = [[1.9,8.15],
        #                 [5.9,17.9],
        #                 [2.7,1.8],
        #                 [9.5,27.0],
        #                 [21.4,27.9]]
        
        self.markerArray = MarkerArray()  
         
        self.marker_pub = self.create_publisher(MarkerArray, 'Goals', QoSProfile(depth=10,durability =1  ))   
          

        self.timer_main = self.create_timer(2, self.timer_callback)
        


    def timer_callback(self): 
        if self.map == None :
            return 
        else:
            distance_to_robots = self.find_distances()         
            self.get_logger().info(str(distance_to_robots)) 
        
            row_ind, col_ind = linear_sum_assignment(np.array(list(distance_to_robots.values())) )
            
            self.visualize_relations(zip(row_ind, col_ind))
            
            self.marker_pub.publish(self.markerArray)

            self.timer_main.cancel()    
    
    
    def find_distances(self):
        dismaps_list=[]
        for pos in self.goal_pos:
            
            goal = [ int( (pos[1]-self.originy)/self.resolution) ,  int( (pos[0]-self.originx)/self.resolution) ] 
            
            dismap = self.create_dismap(self.map, goal)       
            dismaps_list.append(dismap)
        
        distance_to_robots = dict()    
        for robot_idx in range(1,len(self.rob_pos)+1):
            distance_to_robots[robot_idx]=[]
            for dismap in dismaps_list:
                pos =  [ int( (self.rob_pos[robot_idx-1][1]-self.originy)/self.resolution) ,  int( (self.rob_pos[robot_idx-1][0]-self.originx)/self.resolution) ] 
                distance_to_robots[robot_idx].append(dismap[pos[0] * self.WIDTH + pos[1]])        
         
        return distance_to_robots
      
    def map_callback(self,msg):

        self.get_logger().info("map_recieved "+str(msg.info.height)+" , "+str(msg.info.width)+ " , "+str(msg.info.resolution))
        self.get_logger().info("origin "+str(msg.info.origin.position.x)+" , "+str(msg.info.origin.position.y))
        
        self.originx = msg.info.origin.position.x
        self.originy = msg.info.origin.position.y
        self.resolution = msg.info.resolution
        
        self.HEIGHT = msg.info.height
        self.WIDTH = msg.info.width
        self.map = [int(x) for x in msg.data]
        self.get_logger().info("map_copied")
       
    def assign_id(self):
        self.id+=1
        return self.id

    def create_dismap(self, dismap_backup_, goal):
        curr_iter = []
        next_iter = []
        dismap_ = []

        dismap_= dismap_backup_[::]

        iter_ = 0
        curr_iter.append([goal[0], goal[1]])

        dismap_[goal[0] * self.WIDTH + goal[1]] = -500
        
        while len(curr_iter)>0:
            
            for i in range(len(curr_iter)):
                if (  ((curr_iter[i][0] + 1) * self.WIDTH + curr_iter[i][1])>=0 and ((curr_iter[i][0] + 1) * self.WIDTH + curr_iter[i][1])< self.HEIGHT*self.WIDTH  ):
                    if dismap_[(curr_iter[i][0] + 1) * self.WIDTH + curr_iter[i][1]] == 0:
                        dismap_[(curr_iter[i][0] + 1) * self.WIDTH + curr_iter[i][1]] = iter_ + 1
                        next_iter.append([curr_iter[i][0] + 1, curr_iter[i][1]])
                
                
                if (  ((curr_iter[i][0] ) * self.WIDTH + curr_iter[i][1] + 1)>=0 and ((curr_iter[i][0] ) * self.WIDTH + curr_iter[i][1] + 1)< self.HEIGHT*self.WIDTH  ): 
                    if dismap_[curr_iter[i][0] * self.WIDTH + curr_iter[i][1] + 1] == 0:
                        dismap_[curr_iter[i][0] * self.WIDTH + curr_iter[i][1] + 1] = iter_ + 1
                        next_iter.append([curr_iter[i][0], curr_iter[i][1] + 1])
                
                if (  ((curr_iter[i][0] - 1) * self.WIDTH + curr_iter[i][1])>=0 and ((curr_iter[i][0] - 1) * self.WIDTH + curr_iter[i][1])< self.HEIGHT*self.WIDTH  ):
                    if dismap_[(curr_iter[i][0] - 1) * self.WIDTH + curr_iter[i][1]] == 0:
                        dismap_[(curr_iter[i][0] - 1) * self.WIDTH + curr_iter[i][1]] = iter_ + 1
                        next_iter.append([curr_iter[i][0] - 1, curr_iter[i][1]])
               
                
                if (  ((curr_iter[i][0] ) * self.WIDTH + curr_iter[i][1]-1)>=0 and ((curr_iter[i][0] ) * self.WIDTH + curr_iter[i][1]-1)< self.HEIGHT*self.WIDTH  ): 
                    if dismap_[curr_iter[i][0] * self.WIDTH + curr_iter[i][1] - 1] == 0:
                        dismap_[curr_iter[i][0] * self.WIDTH + curr_iter[i][1] - 1] = iter_ + 1
                        next_iter.append([curr_iter[i][0], curr_iter[i][1] - 1])
               
                if (  ((curr_iter[i][0] + 1) * self.WIDTH + curr_iter[i][1] + 1)>=0 and ((curr_iter[i][0] + 1) * self.WIDTH + curr_iter[i][1] + 1)< self.HEIGHT*self.WIDTH  ):
                    if dismap_[(curr_iter[i][0] + 1) * self.WIDTH + curr_iter[i][1] + 1] == 0:
                        dismap_[(curr_iter[i][0] + 1) * self.WIDTH + curr_iter[i][1] + 1] = iter_ + 2
                        next_iter.append([curr_iter[i][0] + 1, curr_iter[i][1] + 1])
               
                
                if (  ((curr_iter[i][0] + 1) * self.WIDTH + curr_iter[i][1] - 1)>=0 and ((curr_iter[i][0] + 1) * self.WIDTH + curr_iter[i][1] - 1)< self.HEIGHT*self.WIDTH  ): 
                    if dismap_[(curr_iter[i][0] + 1) * self.WIDTH + curr_iter[i][1] - 1] == 0:
                        dismap_[(curr_iter[i][0] + 1) * self.WIDTH + curr_iter[i][1] - 1] = iter_ + 2
                        next_iter.append([curr_iter[i][0] + 1, curr_iter[i][1] - 1])
                
                
                if (  ((curr_iter[i][0] - 1) * self.WIDTH + curr_iter[i][1] + 1)>=0 and ((curr_iter[i][0] - 1) * self.WIDTH + curr_iter[i][1] + 1)< self.HEIGHT*self.WIDTH  ):
                    if dismap_[(curr_iter[i][0] - 1) * self.WIDTH + curr_iter[i][1] + 1] == 0:
                        dismap_[(curr_iter[i][0] - 1) * self.WIDTH + curr_iter[i][1] + 1] = iter_ + 2
                        next_iter.append([curr_iter[i][0] - 1, curr_iter[i][1] + 1])  
            
                
                if (  ((curr_iter[i][0] - 1) * self.WIDTH + curr_iter[i][1] - 1 )>=0 and ((curr_iter[i][0] - 1) * self.WIDTH + curr_iter[i][1] - 1 )< self.HEIGHT*self.WIDTH  ): 
                    if dismap_[(curr_iter[i][0] - 1) * self.WIDTH + curr_iter[i][1] - 1] == 0:
                        dismap_[(curr_iter[i][0] - 1) * self.WIDTH + curr_iter[i][1] - 1] = iter_ + 2
                        next_iter.append([curr_iter[i][0] - 1, curr_iter[i][1] - 1])
         
            curr_iter = next_iter[:]
            next_iter =[]
            iter_+=1
            
        dismap_[goal[0] * self.WIDTH + goal[1]] = 0
        return dismap_
    
                     


    def robot_markers(self):
        
        marker_array = MarkerArray()
        index = 1
        for v in self.rob_pos:
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = 3
            marker.id = self.assign_id()
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.pose.position.x = float(v[0])
            marker.pose.position.y = float(v[1])
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker_array.markers.append(marker) 
            
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = 9
            marker.id = self.assign_id()
            marker.scale.x = 0.8
            marker.scale.y = 0.8
            marker.scale.z = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.pose.position.x = float(v[0])
            marker.pose.position.y = float(v[1])+0.5
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.text = "R"+str(index)
            marker_array.markers.append(marker)  
            
            index+=1 
        return marker_array.markers
        
    def goal_markers(self):
        marker_array = MarkerArray()  
        index = 1  
        for v in self.goal_pos:
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = 3
            marker.id = self.assign_id()
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.pose.position.x = float(v[0])
            marker.pose.position.y = float(v[1])
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker_array.markers.append(marker) 
            
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = 9
            marker.id = self.assign_id()
            marker.scale.x = 0.8
            marker.scale.y = 0.8
            marker.scale.z = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.pose.position.x = float(v[0])
            marker.pose.position.y = float(v[1])+0.5
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.text = "G"+str(index)
            marker_array.markers.append(marker)  
            index+=1
            
        return marker_array.markers    
              
    def visualize_relations(self, matrix):
        connection_array = MarkerArray()
        for i, j in matrix:
            self.get_logger().info(f"Robot {i+1} should go to Location {j+1}")
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.LINE_STRIP
            marker.id = self.assign_id()
            marker.scale.x = 0.1
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            p1 = Point()
            p1.x = float(self.rob_pos[i][0])
            p1.y = float(self.rob_pos[i][1])
            p1.z = 0.0 
            p2 = Point()
            p2.x = float(self.goal_pos[j][0])
            p2.y = float(self.goal_pos[j][1])
            p2.z = 0.0 
            marker.points.append(p1)
            marker.points.append(p2)
            connection_array.markers.append(marker)  
              
        self.markerArray.markers+= self.robot_markers() + self.goal_markers() + connection_array.markers    

def main(args=None):
    rclpy.init(args=args)

    node = RobotsToGoals()

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
