class AStar():
    def __init__(self, env):
        self.agent_dict = env.agent_dict
        self.admissible_heuristic = env.admissible_heuristic
        self.is_at_goal = env.is_at_goal
        self.get_neighbors = env.get_neighbors

    
    def search(self, agent_name):
        """
        low level search 
        """
        cnt =0
        initial_state = self.agent_dict[agent_name]["start"]
        step_cost = 1
        
        closed_set = set()
        open_set = {initial_state}

        came_from = {}

        g_score = {} 
        g_score[initial_state] = 0

        f_score = {} 

        f_score[initial_state] = self.admissible_heuristic(initial_state, agent_name)
        # or initializing f_score[initial_state] with 0 as we are considering optimal condition

        while open_set:
            temp_dict = {open_item:f_score.setdefault(open_item, float("inf")) for open_item in open_set}
            current = min(temp_dict, key=temp_dict.get)

# check if at pick up then make the goal as drop, then if reached at drop, change goal location to end position
# return the function and call to reconstrcut path, which terminate the search process for a agent
            if self.is_at_goal(current, agent_name):
                print(current)
                if (current.location.x == self.agent_dict[agent_name]['goal'].location.x and current.location.y == self.agent_dict[agent_name]['goal'].location.y  and current.location == self.agent_dict[agent_name]["drop"].location):
                    # print('drop to end')
                    # self.agent_dict[agent_name]["start"] = self.agent_dict[agent_name]["drop"]
                    self.agent_dict[agent_name]["goal"] = self.agent_dict[agent_name]["end"]
                    print('drop to end')
                    self.agent_dict[agent_name]['check'].location.y =1
                    print(self.agent_dict[agent_name]['check'])
                    return self.reconstruct_path(came_from, current)

                elif (current.location == self.agent_dict[agent_name]['goal'].location and current.location.y == self.agent_dict[agent_name]['goal'].location.y and current.location == self.agent_dict[agent_name]["pick"].location):
                    # print('pick to drop')
                    # self.agent_dict[agent_name]["start"] = self.agent_dict[agent_name]["pick"]
                    self.agent_dict[agent_name]["goal"] = self.agent_dict[agent_name]["drop"]
                    # current.time +=1
                    print('pick to drop')
                    self.agent_dict[agent_name]['check'].location.x =1
                    print(self.agent_dict[agent_name]['check'])
                elif(self.agent_dict[agent_name]['check'].location.x == 1 and self.agent_dict[agent_name]['check'].location.y == 1):
                    return self.reconstruct_path(came_from, current)
                    #return self.reconstruct_path(came_from, current)
# making possible open and closed position
            open_set -= {current}
            closed_set |= {current}

            neighbor_list = self.get_neighbors(current)

            for neighbor in neighbor_list:
                if neighbor in closed_set:
                    continue
                
                tentative_g_score = g_score.setdefault(current, float("inf")) + step_cost

                if neighbor not in open_set:
                    open_set |= {neighbor}
                elif tentative_g_score >= g_score.setdefault(neighbor, float("inf")):
                    continue
                #making the coming from dict, which store points in the motion of a particular agent
                if(agent_name == 'agent0'): print(neighbor, "     " , current)
                came_from[neighbor] = current

                g_score[neighbor] = tentative_g_score
                # removing the heuristics condition from next line
                f_score[neighbor] = g_score[neighbor] 
        return False
    def reconstruct_path(self, came_from, current):
        total_path = [current]
        new_path = []
        last = []
        curr = []
        full = []
        for i in came_from:
            curr.append(i.location)
            last.append(came_from[i].location)
            full.append(i)
            # print('curr', "  ", i.location)
        look_for = current.location
        # print("   ------    ")
        
        for i ,j in enumerate(reversed(curr)):
            # print(i)
            # print(j, "   ", look_for)
            if j == look_for:
                new_path.append(full[len(full)-i-1])
                print("looking for", look_for, "   last",last[len(last) - i-1] , "i - ", i)
                look_for = last[len(last) - i-1]

        print("=================================")
        
        while current in came_from.keys():
            current = came_from[current]
            total_path.append(current)

        
        # # for i, j in enumerate(new_path):

        # return new_path
        return total_path[::-1]

