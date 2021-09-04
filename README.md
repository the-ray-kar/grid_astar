# grid_astar
A-star module for path planning for robots.

Usage
```python
import numpy as np
from c_astar import astar
import matplotlib.pyplot as plt
#create obstacle map
state_low = np.array([0,0])
state_high = np.array([100,100])
start_state = state_low.copy()
goal_state = state_high.copy()
actions = [np.array([0,1]),np.array([1,1]),np.array([1,0]),np.array([-1,1]),np.array([-1,0]),np.array([-1,-1]),np.array([0,-1]),np.array([1,-1])] #list of actions
step_cost = np.array([1, 1.41, 1, 1.41, 1, 1.41, 1, 1.41])

mapp = np.zeros((101,101)) #create map as 2d array. And obstacles aswell.
mapp[20:40,20:40] = 1
mapp[20:40,60:80] = 1
mapp[60:80,20:40] = 1
mapp[50:70,50:80]=1
mapp[80:100,70:90]=1

#append obstacle points for plottinf
xo = []
yo = []
for i in range(100):
    for j in range(100):
        if(mapp[i,j]==1):
            xo.append(i)
            yo.append(j)
plt.xlim([0,101])
plt.ylim([0,101])
plt.scatter(xo,yo)
plt.show()
![image](https://user-images.githubusercontent.com/70949901/132097970-4592a087-64e9-4f54-91db-2b6b9bc7e262.png)

#define required user custom functions for planning
def get_next(state,action):
    next_state = state + action
    return next_state

def compute_heuristic(state):
    global goal_state
    diff = goal_state - state
    h = np.linalg.norm(diff)
    #print("state heuristic",state,"goal_state",goal_state,"heuristic",h)
    return h

def detect_collision(state):
    global mapp, state_low, state_high
    out_of_state = (state>state_high).any() + (state<state_low).any()
    #print(state)
    if(out_of_state):
        return True
    s = tuple(state)
    if(mapp[s]==1):
        return True
    return False    

def goal_condition(state:np.ndarray):
    global goal_state
    return (state==goal_state).all()

#Initialise the planner by providing the user defined functions and other required attributes
planner = astar(start_state=start_state,goal_state=goal_state,get_next_state=get_next,compute_heuristic=compute_heuristic,
                detect_collision=detect_collision,state_low=state_low,state_high=state_high,action_list=actions,step_cost=tep_cost,dis_buckets=[101,101],goal_condition=goal_condition)   

way_points,final_node = planner.compute_shortest_path() #get the shortest path

#plot the path
points = np.array(p)
plt.scatter(points[:,0],points[:,1])
plt.scatter(xo,yo)
plt.show()

![image](https://user-images.githubusercontent.com/70949901/132098087-007039e1-31ee-41ee-8e08-e154a83cc28d.png)


```
