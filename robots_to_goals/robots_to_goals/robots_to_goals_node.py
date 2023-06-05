import rclpy
from rclpy.node import Node
import math
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

import numpy as np
from scipy.optimize import linear_sum_assignment


distance_to_robots_list =[]
all_possibilities = [] 
class RobotsToGoals(Node):
    
    
    def __init__(self):
        super().__init__('robots_to_goals_node')
        
        self.HEIGHT,self.WIDTH,self.originx,self.originy,self.resolution = 0,0,0,0,0
        self.map = None
        self.id = -1
       
        self.subscription = self.create_subscription(OccupancyGrid,"/human/voronoi_map", self.map_callback,QoSProfile(depth=10,durability =1))
        self.subscription = self.create_subscription(MarkerArray,"/human/visualization_marker_array", self.marker_callback,QoSProfile(depth=10,durability =1))
       
       
        self.goal_pos = []

        self.rob_pos = []
        
        
        self.markerArray = MarkerArray()  
         
        self.marker_pub = self.create_publisher(MarkerArray, 'Goals', QoSProfile(depth=10,durability =1  ))   
          
        self.matrix = None
        
        self.timer_main = self.create_timer(2, self.timer_callback)
        


    def timer_callback(self): 
    
        if self.map == None or len(self.goal_pos)==0:
            return 
        elif self.matrix == None:
            distance_to_robots = self.find_distances() 
            #self.get_logger().info(str(distance_to_robots)) 

            #previous method
            # row_ind, col_ind = linear_sum_assignment(np.array(list(distance_to_robots.values())) )
            
            #New method
            row_ind, col_ind = self.assign_goals(distance_to_robots)
            
            self.get_logger().info(str(row_ind)+"  " +str(col_ind))
            self.matrix = tuple(zip(row_ind, col_ind))
            
            self.visualize_relations(self.matrix)
  
            self.marker_pub.publish(self.markerArray)
            
            
        else:
            self.get_logger().info("Sending Goals") 
            self.send_goals()
            
    


    def find_possibilities(self,data,next_ind,Unavailable_indexes,current_possibility,distances):
        if len(Unavailable_indexes) == len(data):
            distances.sort(reverse=True)
            all_possibilities.append([distances,current_possibility])
            return
        
        for x in range(len(data[next_ind])):
            if x in Unavailable_indexes:
                continue
            
            self.find_possibilities(distance_to_robots_list,next_ind+1,Unavailable_indexes+[x],current_possibility+[(next_ind,x)],distances+[distance_to_robots_list[next_ind][x]])
        

    def assign_goals(self,distance_to_robots):
        global distance_to_robots_list,all_possibilities
        
        distance_to_robots_list = [distance_to_robots[i] for i in range(1,len(distance_to_robots)+1)]

        next_ind = 0
        for y in range(len(distance_to_robots_list)):
            possibility=[(next_ind,y)]
            self.find_possibilities(distance_to_robots_list,next_ind+1,[y],possibility[:],[distance_to_robots_list[next_ind][y]])
        
        
     
        all_possibilities.sort(reverse=True)
     
        best_possibility=all_possibilities[-1]
            
        print(best_possibility)    
        return [[robot[0] for robot in best_possibility[1]],[goal[1] for goal in best_possibility[1]]]


    
    def send_goals(self):
        robot_goal_ = NavigateToPose.Goal()
        robot_goal_.pose.header.frame_id = "map"
        robot_goal_.pose.pose.position.z = 0.0
        robot_goal_.pose.pose.orientation.z = 1.0

        for i, j in self.matrix :
            action_client = ActionClient(self, NavigateToPose, '/robot'+str(i+1)+'/navigate_to_pose')
            robot_goal_.pose.header.stamp = self.get_clock().now().to_msg()
            robot_goal_.pose.pose.position.x = self.goal_pos[j][0]
            robot_goal_.pose.pose.position.y = self.goal_pos[j][1]
            action_client.send_goal_async(robot_goal_)  
    
        
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
        
        self.originx = msg.info.origin.position.x
        self.originy = msg.info.origin.position.y
        self.resolution = msg.info.resolution
        
        self.HEIGHT = msg.info.height
        self.WIDTH = msg.info.width
        self.map = [int(x) for x in msg.data]
        self.get_logger().info("map_copied")
        
    def marker_callback(self,msg):
        if not len(self.goal_pos):
            for marker in msg.markers:
                if marker.type == 2: #Type of the goals marker
                    self.goal_pos.append([marker.pose.position.x,marker.pose.position.y])
                if marker.type == 3: #Type of the robot marker
                    self.rob_pos.append([marker.pose.position.x,marker.pose.position.y])    

        self.get_logger().info("marker_array_recieved ")
  
       
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
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
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
              
        self.markerArray.markers+= connection_array.markers
        
 
            
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
