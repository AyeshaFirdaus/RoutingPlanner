#!/usr/bin/env python
# coding: utf-8

# # Implementing a Route Planner
# In this project you will use A\* search to implement a "Google-maps" style route planning algorithm.

# ## The Map

# In[363]:


# Run this cell first!

from helpers import Map, load_map_10, load_map_40, show_map
import math

get_ipython().run_line_magic('load_ext', 'autoreload')
get_ipython().run_line_magic('autoreload', '2')


# ### Map Basics

# In[364]:


map_10 = load_map_10()
show_map(map_10)


# The map above (run the code cell if you don't see it) shows a disconnected network of 10 intersections. The two intersections on the left are connected to each other but they are not connected to the rest of the road network. This map is quite literal in its expression of distance and connectivity. On the graph above, the edge between 2 nodes(intersections) represents a literal straight road not just an abstract connection of 2 cities.
# 
# These `Map` objects have two properties you will want to use to implement A\* search: `intersections` and `roads`
# 
# **Intersections**
# 
# The `intersections` are represented as a dictionary. 
# 
# In this example, there are 10 intersections, each identified by an x,y coordinate. The coordinates are listed below. You can hover over each dot in the map above to see the intersection number.

# In[365]:


map_10.intersections


# **Roads**
# 
# The `roads` property is a list where `roads[i]` contains a list of the intersections that intersection `i` connects to.

# In[366]:


# this shows that intersection 0 connects to intersections 7, 6, and 5
map_10.roads[0] 


# In[367]:


# This shows the full connectivity of the map
map_10.roads


# In[368]:


# map_40 is a bigger map than map_10
map_40 = load_map_40()
show_map(map_40)


# ### Advanced Visualizations
# 
# The map above shows a network of roads which spans 40 different intersections (labeled 0 through 39). 
# 
# The `show_map` function which generated this map also takes a few optional parameters which might be useful for visualizaing the output of the search algorithm you will write.
# 
# * `start` - The "start" node for the search algorithm.
# * `goal`  - The "goal" node.
# * `path`  - An array of integers which corresponds to a valid sequence of intersection visits on the map.

# In[ ]:





# In[369]:


# run this code, note the effect of including the optional
# parameters in the function call.
show_map(map_40, start=5, goal=34, path=[5,16,37,12,34])


# ## The Algorithm
# ### Writing your algorithm
# The algorithm written will be responsible for generating a `path` like the one passed into `show_map` above. In fact, when called with the same map, start and goal, as above you algorithm should produce the path `[5, 16, 37, 12, 34]`. However you must complete several methods before it will work.
# 
# ```bash
# > PathPlanner(map_40, 5, 34).path
# [5, 16, 37, 12, 34]
# ```

# In[370]:


# Do not change this cell
# When you write your methods correctly this cell will execute
# without problems
class PathPlanner():
    """Construct a PathPlanner Object"""
    def __init__(self, M, start=None, goal=None):
        """ """
        self.map = M
        self.start= start
        self.goal = goal
        self.closedSet = self.create_closedSet() if goal != None and start != None else None
        self.openSet = self.create_openSet() if goal != None and start != None else None
        self.cameFrom = self.create_cameFrom() if goal != None and start != None else None
        self.gScore = self.create_gScore() if goal != None and start != None else None
        self.fScore = self.create_fScore() if goal != None and start != None else None
        self.path = self.run_search() if self.map and self.start != None and self.goal != None else None
        
    def get_path(self):
        """ Reconstructs path after search """
        if self.path:
            return self.path 
        else :
            self.run_search()
            return self.path
    
    def reconstruct_path(self, current):
        """ Reconstructs path after search """
        total_path = [current]
        while current in self.cameFrom.keys():
            current = self.cameFrom[current]
            total_path.append(current)
        return total_path
    
    def _reset(self):
        """Private method used to reset the closedSet, openSet, cameFrom, gScore, fScore, and path attributes"""
        self.closedSet = None
        self.openSet = None
        self.cameFrom = None
        self.gScore = None
        self.fScore = None
        self.path = self.run_search() if self.map and self.start and self.goal else None

    def run_search(self):
        """ """
        if self.map == None:
            raise(ValueError, "Must create map before running search. Try running PathPlanner.set_map(start_node)")
        if self.goal == None:
            raise(ValueError, "Must create goal node before running search. Try running PathPlanner.set_goal(start_node)")
        if self.start == None:
            raise(ValueError, "Must create start node before running search. Try running PathPlanner.set_start(start_node)")

        self.closedSet = self.closedSet if self.closedSet != None else self.create_closedSet()
        self.openSet = self.openSet if self.openSet != None else  self.create_openSet()
        self.cameFrom = self.cameFrom if self.cameFrom != None else  self.create_cameFrom()
        self.gScore = self.gScore if self.gScore != None else  self.create_gScore()
        self.fScore = self.fScore if self.fScore != None else  self.create_fScore()

        while not self.is_open_empty():
            current = self.get_current_node()

            if current == self.goal:
                self.path = [x for x in reversed(self.reconstruct_path(current))]
                return self.path
            else:
                self.openSet.remove(current)
                self.closedSet.add(current)

            for neighbor in self.get_neighbors(current):
                if neighbor in self.closedSet:
                    continue    # Ignore the neighbor which is already evaluated.

                if not neighbor in self.openSet:    # Discover a new node
                    self.openSet.add(neighbor)
                
                # The distance from start to a neighbor
                #the "dist_between" function may vary as per the solution requirements.
                if self.get_tenative_gScore(current, neighbor) >= self.get_gScore(neighbor):
                    continue        # This is not a better path.

                # This path is the best until now. Record it!
                self.record_best_path_to(current, neighbor)
        print("No Path Found")
        self.path = None
        return False


# Create the following methods:

# In[371]:


def create_closedSet(self):
    """ Creates and returns a data structure suitable to hold the set of nodes already evaluated"""
    # TODO: return a data structure suitable to hold the set of nodes already evaluated
    return set()


# In[372]:


def create_openSet(self):
    """ Creates and returns a data structure suitable to hold the set of currently discovered nodes 
    that are not evaluated yet. Initially, only the start node is known."""
    if self.start != None:
        # TODO: return a data structure suitable to hold the set of currently discovered nodes 
        # that are not evaluated yet. Make sure to include the start node.
        pas=set()
        pas.add(self.start)
        return (pas)
    
    raise(ValueError, "Must create start node before creating an open set. Try running PathPlanner.set_start(start_node)")


# In[373]:


def create_cameFrom(self):
    """Creates and returns a data structure that shows which node can most efficiently be reached from another,
    for each node."""
    # TODO: return a data structure that shows which node can most efficiently be reached from another,
    # for each node.
    dic={}
    return dic


# In[374]:


def create_gScore(self):
    """Creates and returns a data structure that holds the cost of getting from the start node to that node, for each node.
    The cost of going from start to start is zero."""
    # TODO:  a data structure that holds the cost of getting from the start node to that node, for each node.
    # for each node. The cost of going from start to start is zero. The rest of the node's values should be set to infinity.
    c = {}
    for a in range(len(self.map.intersections)):
        c[a] = float('inf')
    c[self.start] = 0
    return c
    


# In[375]:


def create_fScore(self):
    """Creates and returns a data structure that holds the total cost of getting from the start node to the goal
    by passing by that node, for each node. That value is partly known, partly heuristic.
    For the first node, that value is completely heuristic."""
    # TODO:  a data structure that holds the total cost of getting from the start node to the goal
    # by passing by that node, for each node. That value is partly known, partly heuristic.
    # For the first node, that value is completely heuristic. The rest of the node's value should be 
    # set to infinity.
    dicts={}
    ml=len(self.map.intersections)
    for i in range(ml):
        dicts[i]=float('inf')
    dicts[self.start]=self.heuristic_cost_estimate(self.start)
    return dicts


# In[376]:


def _reset(self):
    self.cameFrom=None
    self.closedSet=None
    self.fScore=None
    self.gScore=None
    self.openSet=None
    self.path=self.run_search() if self.map and self.goal else None

def set_map(self, M):
    """Method used to set map attribute """
    self._reset(self)
    self.start = None
    self.goal = None
    # TODO: Set map to new value. 
    self.map = M


# In[377]:


def set_start(self, start):
    """Method used to set start attribute """
    self._reset(self)
    # TODO: Set start value. Remember to remove goal, closedSet, openSet, cameFrom, gScore, fScore, 
    # and path attributes' values.
    self.cameFrom = None 
    self.closedSet = None 
    self.fScore = None 
    self.gScore = None 
    self.goal=None
    self.openSet =None 
    self.path=None
    self.start=start
    
    
    


# In[389]:


def set_goal(self, goal):
    """Method used to set goal attribute """
    self._reset(self)
    # TODO: Set goal value. 
    self.goal=goal


# In[390]:


def get_current_node(self):
    """ Returns the node in the open set with the lowest value of f(node)."""
    # TODO: Return the node in the open set with the lowest value of f(node).
    dic={}
    for node in self.openSet:
        dic[node]=self.calculate_fscore(node)
    return min(dic, key=dic.get)


# In[391]:


def get_neighbors(self, node):
    """Returns the neighbors of a node"""
    # TODO: Return the neighbors of a node
    return set(self.map.roads[node])


# In[392]:


def get_gScore(self, node):
    """Returns the g Score of a node"""
    # TODO: Return the g Score of a node
    return self.gScore.get(node,0.0)


# In[396]:


def get_tenative_gScore(self, current, neighbor):
    """Returns the tenative g Score of a node"""
    # TODO: Return the g Score of the current node 
    # plus distance from the current node to it's neighbors
    g=self.get_gScore(current)
    d=self.distance(current,neighbor)
    return (g+d)
    


# In[397]:


def is_open_empty(self):
    """returns True if the open set is empty. False otherwise. """
    # TODO: Return True if the open set is empty. False otherwise.
    return (not bool(len(self.openSet)))


# In[398]:


def distance(self, node_1, node_2):
    """ Computes the Euclidean L2 Distance"""
    # TODO: Compute and return the Euclidean L2 Distance
    
    m=self.map.intersections
    
    l1=m[node_1][0]-m[node_2][0]
    l2=m[node_1][1]-m[node_2][1]
    
    mul1 = (l1**2)
    mul2=(l2**2)
    sums=mul1+mul2
    results=(sums**0.5)
    
    return results


# In[399]:


def heuristic_cost_estimate(self, node):
    """ Returns the heuristic cost estimate of a node """
    # TODO: Return the heuristic cost estimate of a node
    
    d=self.distance(node,self.goal)
    
    return d
    


# In[400]:


def calculate_fscore(self, node):
    """Calculate the f score of a node. """
    # TODO: Calculate and returns the f score of a node. 
    # REMEMBER F = G + H
    
    G=self.get_gScore(node)
    H=self.heuristic_cost_estimate(node)
    F=G+H
    return F
    


# In[401]:


def record_best_path_to(self, current, neighbor):
    """Record the best path to a node """
    # TODO: Record the best path to a node, by updating cameFrom, gScore, and fScore

    
    c=current
    gts=self.get_tenative_gScore(current,neighbor)
    calf=self.calculate_fscore(current)
    
    self.cameFrom[neighbor]=c
    self.gScore[neighbor]=gts
    self.fScore[current]=calf


# In[402]:


PathPlanner.create_closedSet = create_closedSet
PathPlanner.create_openSet = create_openSet
PathPlanner.create_cameFrom = create_cameFrom
PathPlanner.create_gScore = create_gScore
PathPlanner.create_fScore = create_fScore
PathPlanner._reset = _reset
PathPlanner.set_map = set_map
PathPlanner.set_start = set_start
PathPlanner.set_goal = set_goal
PathPlanner.get_current_node = get_current_node
PathPlanner.get_neighbors = get_neighbors
PathPlanner.get_gScore = get_gScore
PathPlanner.get_tenative_gScore = get_tenative_gScore
PathPlanner.is_open_empty = is_open_empty
PathPlanner.distance = distance
PathPlanner.heuristic_cost_estimate = heuristic_cost_estimate
PathPlanner.calculate_fscore = calculate_fscore
PathPlanner.record_best_path_to = record_best_path_to


# In[403]:


planner = PathPlanner(map_40, 5, 34)
path = planner.path
if path == [5, 16, 37, 12, 34]:
    print("great! Your code works for these inputs!")
else:
    print("something is off, your code produced the following:")
    print(path)


# ### Testing your Code
# If the code below produces no errors, your algorithm is behaving correctly. You are almost ready to submit! Before you submit, go through the following submission checklist:
# 
# **Submission Checklist**
# 
# 1. Does my code pass all tests?
# 2. Does my code implement `A*` search and not some other search algorithm?
# 3. Do I use an **admissible heuristic** to direct search efforts towards the goal?
# 4. Do I use data structures which avoid unnecessarily slow lookups?
# 
# When you can answer "yes" to all of these questions, submit by pressing the Submit button in the lower right!

# In[404]:


from test import test

test(PathPlanner)


# ## Questions
# 
# **Instructions**  Answer the following questions in your own words. We do not you expect you to know all of this knowledge on the top of your head. We expect you to do research and ask question. However do not merely copy and paste the answer from a google or stackoverflow. Read the information and understand it first. Then use your own words to explain the answer.

# - How would you explain A-Star to a family member(layman)?
# 
# ** ANSWER **:If you have more than one path to go to your destination.One path will take more time and more effort,where as the other path is shortest as well as nearest which will not take more effort but it does through a jungle .The best way with minimum time and less efford we can find through A-star.

# - How does A-Star search algorithm differ from Uniform cost search? What about Best First search?
# 
# ** ANSWER **:Uniform cost search can be used for simple graphs where the cost is the priority ,you can get minimum cost.
# Where as best first search gives you the shortest path .
# A-star give you both shortest path as well as minimum cost.

# - What is a heuristic?
# 
# ** ANSWER **:Heuristic is difference of current state with respect to the goal state.

# - What is a consistent heuristic?
# 
# ** ANSWER **:Consistent heuristic is the value we get when the estimated value is always less than or equal to the sum of  neighboring node of the goal and step cost of reaching that neighbor.

# - What is a admissible heuristic? 
# 
# ** ANSWER **:Admissible heuristic is the value which is never over estimated or never higher than the lowest possible cost from the current point in the path.

# - ___ admissible heuristic are consistent.
# *CHOOSE ONE*
#     - All
#     - Some
#     - None
#     
# ** ANSWER **:Some

# - ___ Consistent heuristic are admissible.
# *CHOOSE ONE*
#     - All
#     - Some
#     - None
#     
# ** ANSWER **:All
