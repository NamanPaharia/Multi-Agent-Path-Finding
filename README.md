# Multi-Agent-Path-Finding-AIFA
## Problem Statement


In a factory setting, usage of robots to automate tasks like pick up and delivery are quite prevalent in recent times. In such scenarios, the collision of the robots is undesirable and might lead to higher maintenance costs. Furthermore, in the given problem statement, the robot has to “pick up” a task/item from the pick up (source) location, after starting in its initial start position and then eventually after “dropping” the task/item to a drop location (destination), the robot has to finally move to its end position. There are obstacles as well where the robot cannot move and the source and destination locations can accommodate multiple robots simultaneously.


## Algorithms Used
### 1. A* - 

In order to find the shortest path between an initial and Final state, A* algorithm was used - 

It has 3 components/parameters - 

(a) g : Cost incurred in moving from the start state to the current state (it also takes into account all the costs incurred in the path). In pur codes we have taken it as the manhattan distance between two consecutive states

(b) h : The "heuristic value", it is the expected cost of moving from current cell to the final cell, we have taken it to be the euclidean distance between the two in our codes (the ones using heuristics, otherwise for optimal it is taken to be 0)

(c) f : The sum of g and h, depending on which, a node is explored (if the sum is minimum for that node in the open list)

### 2. CBS - 

As explained by the creators of this algorithm - *"CBS (Conflict Based Search) is a two-level algorithm that does not convert the problem into the single ‘joint agent’ model. At the high level, a search is performed on a Conflict Tree (CT) which is a tree based on conflicts between individual agents. Each node in the CT represents a set of constraints on the motion of the agents. At the low level, fast single-agent searches are performed to satisfy the constraints imposed by the high level CT node. In many cases this two-level formulation enables CBS to examine fewer states than Astar while still maintaining optimality"* **(Guni Sharon, Roni Stern, Ariel Felner, Nathan R. Sturtevant,Conflict-based search for optimal multi-agent pathfinding,Artificial Intelligence)**


### 3. TCBS -

For Dynamic task allocation, TCBS (Task Conflict Based Search) algorithm by C. Henkel, J. Abbenseth and M. Toussaint can be used. This algorithm  searches on the    level of task assignments for all agents. Given a configuration of task assignments and neglecting collision constraints. It uses standard single-agent path finding to compute the corresponding optimal paths that connect the agents configurations to the start and the start to the goal configurations
of all assigned tasks. . To also account for the path collision constraints the approach of conflict-based search is used

## Requirements

For installing requirements run - 
```

pip install -r requirements.txt

```

## Results
<img src="https://user-images.githubusercontent.com/45457551/112757854-081a9f80-9009-11eb-9110-78eb565b9868.PNG" width="20" height="20" />   - Pick-up                              <img src="https://user-images.githubusercontent.com/45457551/112757928-621b6500-9009-11eb-81c5-93afea6ad0b3.PNG" width="20" height="20" />   - Drop Location                              <img src="https://user-images.githubusercontent.com/45457551/112757878-2c767c00-9009-11eb-97b5-a8f6c6c16b7d.PNG" width="20" height="20" />   - Destination                              <img src="https://user-images.githubusercontent.com/45457551/112757958-7eb79d00-9009-11eb-979e-b58585bc2cd5.PNG" width="20" height="20" />   - Obstacles





#Test 1 (2 agents)

![result1](https://user-images.githubusercontent.com/45457551/112756617-aad01f80-9003-11eb-808b-2aa26d9c786b.gif)


#Test 2 (3 agents)

![result2](https://user-images.githubusercontent.com/45457551/112756689-08fd0280-9004-11eb-81ba-fb7e0712cd85.gif)



## References

1. Guni Sharon, Roni Stern, Ariel Felner, Nathan R. Sturtevant, Conflict-based search for optimal multi-agent pathfinding, Artificial Intelligence, Volume 219,
2015, Pages 40-66, ISSN 0004-3702, https://doi.org/10.1016/j.artint.2014.11.006.

2. C. Henkel, J. Abbenseth and M. Toussaint, "An Optimal Algorithm to Solve the Combined Task Allocation and Path Finding Problem," 2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Macau, China, 2019, pp. 4140-4146, doi: 10.1109/IROS40897.2019.8968096.
