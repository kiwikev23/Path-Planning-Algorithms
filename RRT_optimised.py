# -*- coding: utf-8 -*-
"""
Created on Sat Nov 23 17:20:37 2023

@author: Kevin Darren
"""

import random
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.transforms import Bbox
from matplotlib.path import Path
import math
import time

class MapGenerator:
    
    def __init__(self, num_of_obstacles, width_grid, height_grid):
        self.num_of_obstacles = num_of_obstacles 
        self.width_grid = width_grid
        self.height_grid = height_grid
        self.grid_points = [(i,j) for i in range(3,self.width_grid-3) for j in range(3,self.height_grid-3)]     #all the viable grid points for center of obsatcles
        self.bbox = []
        
    #rectangular obsatcle generation
    def generate_obstacles(self):
       rectangles = []  #list of valid obstacles
       grid_points_avail = self.grid_points
    
       for i in range(self.num_of_obstacles):
           valid_obstacle = False   #flag to check if the valid from the overlap logic 
           while not valid_obstacle:
               x, y = random.choice(grid_points_avail)  #center of the obstacle
               w, h = random.randint(1, 2), random.randint(1, 2)  #width and height of the obstacle
               bbox_candidate = Bbox.from_bounds(x - w / 2, y - h / 2, w, h)    #creating bbox for the obstacle rectangle
    
               overlap = any(bbox_candidate.overlaps(existing_bbox) for existing_bbox in self.bbox)     #flag to check if 2 bboxes overlap
    
               if not overlap:
                   rect = (x - w / 2, y - h / 2, w, h)
                   rectangles.append(rect)
                   self.bbox.append(bbox_candidate)
                   valid_obstacle = True
               
       return rectangles

    
        
        
        
class RRT(MapGenerator):
    
    def __init__(self, num_of_obstacles, width_grid, height_grid, start_node, end_point):
        super().__init__(num_of_obstacles, width_grid, height_grid)
        self.start_node = start_node
        self.end_point = end_point
        self.nodes = {self.start_node: []}  #dictionary with the keys being the parent nodes, values being the children nodes
        self.end_node = self.start_node     #intialising end node as start node 
        self.iterations = 0
        
    #function to find if a path length (path between 2 nodes) hit an obstacle
    def obstacle_hit(self, p1, p2):
        hits = False
        path = Path([p1, p2])
        for i in range(self.num_of_obstacles):
            n = 0
            if path.intersects_bbox(self.bbox[i]):
                n = n + 1
                if n >= 1:
                    hits = True
                    break
        return hits
    
    #function to find if the end node is in the end goal region for loop termination
    def endpoint_hit(self, p):
        hits = False
        if ((self.end_point[0] - 0.5) <= p[0] <= (self.end_point[0] + 0.5)) and (
                (self.end_point[1] - 0.5) <= p[1] <= (self.end_point[1] + 0.5)):    #end goal region is modelled as a rectangle with the end goal as the center
            hits = True

        return hits
        
    
    #function to find the closest node from the randomly sampled points
    def find_closest_node(self, p1):
        closest = [math.inf, []]
        for key in self.nodes.keys():
            distance = (key[0] - p1[0]) ** 2 + (key[1] - p1[1]) **2 #not using sqrt since it takes a bit longer
            if distance < closest[0]:
                closest = [distance, key]
            
                
        return closest
                 
                
                    
                
        
    #function for conducting RRT
    def path_planning(self):
        while not self.endpoint_hit(self.end_node):
      
            rand_sample = (random.choice(np.arange(0,10,0.1)),random.choice(np.arange(0,10,0.1))) #sampling a random point in the grid
            closest_node = self.find_closest_node(rand_sample)[1]
            multiplier = random.choice(np.arange(0.3,0.6,0.1)) #randomising the path length between 2 nodes

            dir_vec = np.array(rand_sample) - closest_node #finding the direction of the sampled point
            new_node = list(dir_vec/np.linalg.norm(dir_vec)) 
         
                
            new_node = (multiplier*round(new_node[0],2) + closest_node[0], multiplier*round(new_node[1],2) + closest_node[1])  #new node creation 
            closest_node = tuple(closest_node)
            
       
            #only generate the node if it doesnt hit an obstacle and it's within the grid bounds
            if not self.obstacle_hit(closest_node, new_node) and (0 < new_node[0] < self.width_grid) and (0 < new_node[1] < self.height_grid):

                self.nodes[new_node] = []       #adding as a new parent node (key)
                self.nodes[closest_node].append(new_node) #adding as a child node to the intial parent node (value)
                plt.plot([closest_node[0], new_node[0]], [closest_node[1], new_node[1]], color='black', linewidth = 0.3)
                plt.scatter(new_node[0], new_node[1], color='black')
                self.iterations += 1
                self.end_node = new_node
                


    #function to bactrack the found path and plot it
    def plot_path_plan(self):

        current_node = self.end_node
        self.path = [current_node]
    

        while current_node != self.start_node:

            for parent, children in self.nodes.items():
                if current_node in children:
                    self.path.append(parent)
                    current_node = parent
                    break
    
        
        self.path.reverse()     #reversing the list of nodes
    
        for i in range(len(self.path) - 1):
            plt.plot([self.path[i][0], self.path[i+1][0]], [self.path[i][1], self.path[i+1][1]], color='blue', linewidth=1.5)
    
     
    #function to create a more optimised path from the given path       
    def optimised_path_plan(self):
        
        current_node = self.path[0]
        i = 0
        while True:
            if i < len(self.path):
                if self.obstacle_hit(current_node, self.path[i]): #if 2 nodes when directly joined, do not hit the obstacle, it is bypassed and will not travel the nodes in between
                    plt.plot([current_node[0], self.path[i-1][0]], [current_node[1], self.path[i-1][1]], color='green', linewidth=2)
                    current_node = self.path[i]
                
                else:
                    i += 1
                    
            else:
                plt.plot([current_node[0], self.path[i-1][0]], [current_node[1], self.path[i-1][1]], color='green', linewidth=2)
                break
                
            
        
    
    def run(self):

        obstacles = self.generate_obstacles()

        fig, ax = plt.subplots()
        
        #plotting the start and end points
        plt.scatter(self.start_node[0], self.start_node[1], color='red')
        plt.scatter(self.end_point[0], self.end_point[1], color='red')
        
        #plotting the end goal region rectangle
        end_rect = plt.Rectangle((self.end_point[0] - 0.5, self.end_point[1] - 0.5), 1, 1, color='blue', alpha=0.5)
        ax.add_patch(end_rect)

    
        #plotting obstacles
        for obstacle in obstacles:
            rect = plt.Rectangle((obstacle[0], obstacle[1]), obstacle[2], obstacle[3], color='gray', alpha=0.5)
            ax.add_patch(rect)

        plt.xlim(0, self.width_grid)
        plt.ylim(0, self.height_grid)
        
        intial = time.time()
        self.path_planning()
        final = time.time()
        
        print(f"Net time: {final-intial}")  #time taken for RRT to calculate the path
        
        self.plot_path_plan()
        self.optimised_path_plan()
        
        
    
    
def main():
    start_node = (2, 9)
    end_point = (9, 1)
    rrt = RRT(3, 10, 10, start_node, end_point)
    rrt.run()
    
    
if __name__ == "__main__":
    main()
