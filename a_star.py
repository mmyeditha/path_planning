import math
from heapq import heappush

class State:
    def __init__(self, x, y, z, roll, pitch, yaw, angles, true_cost, parent=None):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.yaw = yaw
        self.angle_list = angles
        self.num_dof = len(angles)
        self.parent = parent
        self.g = 0
        self.true_cost = true_cost
        
    def conf(self):
        """
        Return 'configuration' of a state as according to the paper idfk
        """
        return None

    def g(self):
        """

        """

def a_star_graph_search(start, goal, heuristic):
    """
    start and goal: 3-elem list of x, y, z
    """
    closed = set()
    OPEN = [] # heap
    distance = {start: 0}
    min_state = 0
    x,y,z = start
    initial_state = State(x,y,z,roll, pitch, yaw, angles)
    initial_state.g = c(start, goal)
    while f(goal) > min_state:
        state = frontier.pop()
        if state in closed:
            continue
        else:
            if true_cost(state):
                closed.add(state)
                new_states = get_successors(state)
                for new_state in new_states:
                    if new_state not in closed:
                        new_state.parent = state
                        new_state.g = state.g + c(new_state.parent, new_state)
                        expand_elem = True
                        if expand_elem:
                            # f(s') = g(s') + epsilon*h(s')
                            heappush(OPEN, state)
            else:
                true_cost_calc = getTrueCost(state.parent, state)
                if not math.isinf(true_cost_calc) and not math.isnan(true_cost_calc):
                    state.true_cost = True
                    # g(s) = g(s.parent) + true_cost_calc
                    if should_open(OPEN, state):
                        # f(s) = g(s) + epsilon*h(s)
                        heappush(OPEN, state)
                    

    return None

def reconstruct_path(came_from, start, end):
    """
    came_from = {'b': 'a', 'c': 'a', 'd': 'c', 'e': 'd', 'f': 'd'}
    reconstruct_path(came_from, 'a', 'e')
    ['a', 'c', 'd', 'e']
    """
    reverse_path = [end]
    while end != start:
        end = came_from[end]
        reverse_path.append(end)
    return list(reversed(reverse_path))

def get_successors(state):
    """
    Retrieve all successors of a node in a graph
    """
    return None

def cost(u, v):
    """
    cost function from vertex u to v 

    c(u,v)
    """
    return None

def g(s):
    """
    cheapest path from s_start to s found by algo so far
    """

def heuristic(s):
    '''
    Consistent heuristic, provides underestimate of distance to goal
    Satisfies triangle inequality
    '''
    
def func(s):
    """
    Defined in the paper as 'f(s)'
    """
    return g(s)+heuristic(s)

def true_cost(s):
    """
    check if a state's cost is true
    """
# example state vector: [x, y, z, roll. pitch. yaw, armid, graspid, phi_1, ..., phi_7] (for 7 DOF arm)

def should_open(current_states, state):
    for elem in current_states:
        if elem.conf == state.conf and elem.true_cost and elem.g() < state.g(): 
            # formalize how g is represented as these functions
            # currently g is gonna be a function, make sure that g is calculated the same every time
            # if not g is an instance variable that is set?
            return False
    return True