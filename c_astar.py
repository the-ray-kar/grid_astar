import numpy as np
import time
import heapq #for proriety que implementation

class node:

    def __init__(self,state):
        self.parent = None #pointer to parent
        self.state = state 
        self.g_cost = float('inf')
        self.f_cost = 0.0
        self.is_in_list = False #use a variable to point whether its in list. This avoids searching in list.
        self.keyself = [0,0,self] # [f_cost, index, self_pointer]

class P_queue:

    def __init__(self):
        self.Q = []
        self.count = 0 #this is a counter. It avoids error when two f_costs are same and preventing error while comparing pointers.

    def push(self, Node:node):
        Node.keyself[0]=Node.f_cost
        Node.keyself[1] = self.count
        self.count+=1
        heapq.heappush(self.Q, Node.keyself)
        Node.is_in_list = True #it now present in list
    
    def pop(self):
        poped_node = heapq.heappop(self.Q)
        poped_node[2].is_in_list = False #the last one in the list is the node object
        return poped_node[2]

    def remove(self, Node:node):
        self.Q.remove(Node.keyself) #remove the Node
        heapq.heapify(self.Q) #convert to heap again. NlogN
        Node.is_in_list = False #its no longer in list
    
    def get_length(self):
        return len(self.Q)

    def get(self,index):
        return self.Q[index]




class astar:

    def __init__(self, start_state:np.ndarray, goal_state:np.ndarray, 
                get_next_state, compute_heuristic, detect_collision,state_low:np.ndarray,
                state_high:np.ndarray,action_list:list,step_cost:list,dis_buckets:list,goal_condition = None):

            self.has_computed_path = False
            self.start_state = start_state
            self.goal_state = goal_state
            self.get_next_state = get_next_state
            self.compute_heuristic = compute_heuristic 
            self.detect_collision = detect_collision
            self.goal_condition = goal_condition
            self.action_list = action_list #list of actions that are inputs to get_next_state
            self.step_cost = step_cost #cost from one state to another for actions in action list in same order

            self.state_low = np.array(state_low)
            self.state_high = np.array(state_high) 
            
         
            self.G = None #initialise empty grid

            dis_buckets = np.array(dis_buckets)
            
            self.discrete_levels = (self.state_high-self.state_low)/(dis_buckets-1) #converter
            dis_buckets = tuple(dis_buckets)
            self.dis_buckets = dis_buckets 
            self.G = np.empty(dis_buckets,dtype=node) #create an empty grid
            self.G.flat= [node(None) for _ in self.G.flat] #store in numpy array with no state values
            print("Created a grid of size",dis_buckets,self.G.shape)

            self.open_list = P_queue() #make an open list.

            

            
    def get_grid_state(self,state): #convert continous state to grid indices
        #print(state,self.state_low,self.discrete_levels)
        state = (state - self.state_low)/self.discrete_levels
        return tuple(state.astype(int))

    def push_state(self,state):

        state_index = self.get_grid_state(state) #get state as index

        self.open_list.push(self.G[state_index])  #finally add in open list

    def get_children(self,parent:node):
        parent_state = parent.state
        #print("parent_state",parent_state)
        children_nodes = [] #empty list
        
        for action,step_cost in zip(self.action_list,self.step_cost):
            child_state = self.get_next_state(parent_state,action)
    
            if(self.detect_collision(child_state)):
                continue #its colliding
            child_index = self.get_grid_state(child_state)
            #print(child_state,child_index)
            child_node = self.G[child_index]
            child_node.state = child_state
            #print(child_node.g_cost,parent.g_cost)
            if(child_node.g_cost>(parent.g_cost+step_cost)):

                child_node.parent = parent #update the parent.
                child_node.g_cost = parent.g_cost + step_cost #update g_cost
                child_node.f_cost = self.compute_heuristic(child_state) + child_node.g_cost #update f_cost
                if(not child_node.is_in_list):
                    children_nodes.append(child_node) #add the node 
            

            


        return children_nodes

    
    def get_waypoints(self,p:node):
        waypoints = []
        while p.parent: #iterate backward using heirarchy
            waypoints.append([p.state[0],p.state[1]])
            p = p.parent
        
        return list(reversed(waypoints))


    #Use the A-star algorithm.
    def compute_shortest_path_grid(self):

        state_index = self.get_grid_state(self.start_state) #get state as index
        start_node = self.G[state_index]
        start_node.state = self.start_state
        start_node.g_cost = 0 #set g_cost to zero for start state
        start_node.f_cost = self.compute_heuristic(start_node.state) #update f_cost.
        self.push_state(self.start_state) #add start state to open list
        max_s = 0
        while(self.open_list.get_length()!=0):

            #for n in self.open_list.Q:
            #    print("state",n[2].state,"gcost",n[2].g_cost,"fcost",n[2].f_cost)

            

            poped= self.open_list.pop() #pop node with lowest f_cost
            
            #print("poped",poped.state)

            if(poped.state[0]>max_s):
                max_s = poped.state[0]
                print("Percent completed",max_s/self.goal_state[0]*100)
            if(self.goal_condition(poped.state)):
                #print(poped.parent)
                self.has_computed_path=True
                return self.get_waypoints(poped),poped #return way-points
            


            children = self.get_children(poped) #get feasible child nodes using all actions and grow the tree

            for child in children:
                self.push_state(child.state) #add the children to open list. 


        print("Path cannot be found")
        return False 


   
            
    def compute_shortest_path(self):
       if(self.has_computed_path):
           print("Path already computed, re_computing for new changes if any")
           dis_buckets = self.dis_buckets
           self.G = np.empty(dis_buckets,dtype=node) #create an empty grid
           self.G.flat= [node(None) for _ in self.G.flat] #store in numpy array with no state values
           print("Created a grid of size",dis_buckets,self.G.shape)
           self.open_list = P_queue() #make an open list.
           self.has_computed_path = False

           
       return self.compute_shortest_path_grid()
       




        








            
        
        


        







                

                
    

