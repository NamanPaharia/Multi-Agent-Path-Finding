import os
import pandas
import sys
sys.path.insert(0, '../')
import argparse
import yaml
from math import fabs
from itertools import combinations
from copy import deepcopy
import numpy
from a_star import AStar


class Environment(object):
    def __init__(self, dimension, agents, obstacles):
        self.dimension = dimension
        self.obstacles = obstacles

        self.agents = agents
        self.agent_dict = {}

        self.make_agent_dict()

        self.constraints = Constraints()
        self.constraint_dict = {}

        self.a_star = AStar(self)

    def get_neighbors(self, state):
        neighbors = []

        # Wait action
        n = State(state.time + 1, state.location)
        if self.state_valid(n):
            neighbors.append(n)
        # Up action
        n = State(state.time + 1, Location(state.location.x, state.location.y+1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Down action
        n = State(state.time + 1, Location(state.location.x, state.location.y-1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Left action
        n = State(state.time + 1, Location(state.location.x-1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Right action
        n = State(state.time + 1, Location(state.location.x+1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        return neighbors


    def get_first_conflict(self, solution):
        max_t = max([len(plan) for plan in solution.values()])
        result = Conflict()
        for t in range(max_t):
            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1 = self.get_state(agent_1, solution, t)
                state_2 = self.get_state(agent_2, solution, t)
                if state_1.is_equal_except_time(state_2):
                    result.time = t
                    result.type = Conflict.VERTEX
                    result.location_1 = state_1.location
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    return result

            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1a = self.get_state(agent_1, solution, t)
                state_1b = self.get_state(agent_1, solution, t+1)

                state_2a = self.get_state(agent_2, solution, t)
                state_2b = self.get_state(agent_2, solution, t+1)

                if state_1a.is_equal_except_time(state_2b) and state_1b.is_equal_except_time(state_2a):
                    result.time = t
                    result.type = Conflict.EDGE
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    result.location_1 = state_1a.location
                    result.location_2 = state_1b.location
                    return result
        return False

    def create_constraints_from_conflict(self, conflict):
        constraint_dict = {}
        if conflict.type == Conflict.VERTEX:
            v_constraint = VertexConstraint(conflict.time, conflict.location_1)
            constraint = Constraints()
            constraint.vertex_constraints |= {v_constraint}
            constraint_dict[conflict.agent_1] = constraint
            constraint_dict[conflict.agent_2] = constraint

        elif conflict.type == Conflict.EDGE:
            constraint1 = Constraints()
            constraint2 = Constraints()

            e_constraint1 = EdgeConstraint(conflict.time, conflict.location_1, conflict.location_2)
            e_constraint2 = EdgeConstraint(conflict.time, conflict.location_2, conflict.location_1)

            constraint1.edge_constraints |= {e_constraint1}
            constraint2.edge_constraints |= {e_constraint2}

            constraint_dict[conflict.agent_1] = constraint1
            constraint_dict[conflict.agent_2] = constraint2

        return constraint_dict

    def get_state(self, agent_name, solution, t):
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        else:
            return solution[agent_name][-1]

    def state_valid(self, state):
        return state.location.x >= 0 and state.location.x < self.dimension[0] \
            and state.location.y >= 0 and state.location.y < self.dimension[1] \
            and VertexConstraint(state.time, state.location) not in self.constraints.vertex_constraints \
            and (state.location.x, state.location.y) not in self.obstacles

    def transition_valid(self, state_1, state_2):
        return EdgeConstraint(state_1.time, state_1.location, state_2.location) not in self.constraints.edge_constraints

    def is_solution(self, agent_name):
        pass

    # function to add heuristics to loss function
    def admissible_heuristic(self, state, agent_name):
        goal = self.agent_dict[agent_name]["goal"]
        if(self.agent_dict[agent_name]['check'].location.y ==1):
            # return fabs(state.location.x - goal.location.x) + fabs(state.location.y - goal.location.y) 
        if(self.agent_dict[agent_name]['check'].location.x ==1):
            x = fabs(state.location.x - goal.location.x) + fabs(state.location.y - goal.location.y)
            x+= fabs(self.agent_dict[agent_name]["drop"].location.x - self.agent_dict[agent_name]["end"].location.x) + fabs(self.agent_dict[agent_name]["drop"].location.y - self.agent_dict[agent_name]["end"].location.y)
            # return x
        # return fabs(state.location.x - goal.location.x) + fabs(state.location.y - goal.location.y)
        return 0
    

    # check if at goal
    def is_at_goal(self, state, agent_name):
        # print(state.location)
        # print(goal_state.location)
        # if(state.location == self.agent_dict[agent_name]['goal'].location and state.location== self.agent_dict[agent_name]["pick"].location):
        #     print('works')
        # if(agent_name == 'agent1'):
        #     print(state.location)
        # if (state.location.x == self.agent_dict[agent_name]['goal'].location.x and state.location.y == self.agent_dict[agent_name]['goal'].location.y  and state.location== self.agent_dict[agent_name]["drop"].location ):
        #     # print('drop to end')
        #     self.agent_dict[agent_name]["start"] = self.agent_dict[agent_name]["drop"]
        #     self.agent_dict[agent_name]["goal"] = self.agent_dict[agent_name]["end"]

        # if (state.location.x == self.agent_dict[agent_name]['goal'].location.x and state.location.y == self.agent_dict[agent_name]['goal'].location.y and state.location== self.agent_dict[agent_name]["pick"].location):
        #     # print('pick to drop')
        #     self.agent_dict[agent_name]["start"] = self.agent_dict[agent_name]["pick"]
        #     self.agent_dict[agent_name]["goal"] = self.agent_dict[agent_name]["drop"]
        
        # if(agent_name == 'agent1'):
        #     print(state.location)
        goal_state = self.agent_dict[agent_name]["goal"]
        start_state = self.agent_dict[agent_name]["start"]
        #print(state.is_equal_except_time(goal_state))
        # if(state.is_equal_except_time(goal_state)):
        #     if(self.agent_dict[agent_name]['start'].location.x != self.agent_dict[agent_name]['goal'].location.x and self.agent_dict[agent_name]['start'].location.y != self.agent_dict[agent_name]['goal'].location.y ):
        #         self.agent_dict[agent_name]['start'].location.x = self.agent_dict[agent_name]['goal'].location.x
        #         self.agent_dict[agent_name]['start'].location.y = self.agent_dict[agent_name]['goal'].location.y
        #         self.agent_dict[agent_name]["goal"].location.x = 9
        #         self.agent_dict[agent_name]["goal"].location.y = 9
        # if(agent_name == "agent1" and state.location.x == self.agent_dict[agent_name]['goal'].location.x and state.location.y == self.agent_dict[agent_name]['goal'].location.y and state.location.x!=9 and state.location.y!=9):
        #     self.agent_dict[agent_name]['start'].location.x = self.agent_dict[agent_name]['goal'].location.x
        #     self.agent_dict[agent_name]['start'].location.y = self.agent_dict[agent_name]['goal'].location.y
        #     self.agent_dict[agent_name]["goal"].location.x = 9
        #     self.agent_dict[agent_name]["goal"].location.y = 9
        # if(state.is_equal_except_time(goal_state)):
        #     print("agent: ", agent_name)
        #     print("current-location - ",state.location )
        #     print("end-location - ",goal_state.location )
        #     print("start-location - ",start_state.location )
        #     print(" ")
        
        return state.is_equal_except_time(goal_state)

    def make_agent_dict(self):
        for agent in self.agents:
            start_state = State(0, Location(agent['start'][0], agent['start'][1]))
            pick_state = State(0, Location(agent['pick'][0], agent['pick'][1]))
            drop_state = State(0, Location(agent['drop'][0], agent['drop'][1]))
            end_state = State(0, Location(agent['end'][0], agent['end'][1]))
            goal_state = State(0, Location(agent['pick'][0], agent['pick'][1]))
            check = State(0,Location(agent['check'][0],agent['check'][1])) 
            # self.agent_dict.update({agent['name']:{'start':start_state, 'goal':goal_state}})
            self.agent_dict.update({agent['name']:{'start':start_state, 'pick':pick_state, 'drop':drop_state, 'end':end_state, 'goal':goal_state, "check":check}})

    def compute_solution(self):
        solution = {}
        for agent in self.agent_dict.keys():
            self.constraints = self.constraint_dict.setdefault(agent, Constraints())
            local_solution = self.a_star.search(agent)
            if not local_solution:
                return False
            solution.update({agent:local_solution})
        return solution

    def compute_solution_cost(self, solution):
        return sum([len(path) for path in solution.values()])
