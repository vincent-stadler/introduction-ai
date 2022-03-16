from datetime import time

import pancake_problem

from pancake_problem import PancakeProblem
from queue import PriorityQueue
from search import Search, SearchNode


class WeightedAStarSearch(Search):
    name = "weighted-astar"

    def __init__(self, search_problem, weight, **kwargs):
        super().__init__(search_problem, **kwargs)
        self.w = weight
        if weight == 0:
            self.name = "uniform-cost"
        elif weight == 1:
            self.name = "astar"

    def search(self):
        # early goal test for initial state
        p = self.search_problem
        if p.is_goal(p.initial_state):
            return [p.initial_state], 0

        # enqueue initial state
        frontier = PriorityQueue() # store here 2 element tuples with value of the evaluation function f = g + w * h and its state
        h = p.h(p.initial_state) # g is 0 for initial state, therefore f= 0 + h
        f = h
        first_tup = (SearchNode(p.initial_state, None, 0), f)
        frontier.put(first_tup)     # frontier will be sorted after first entry of tuple
        self.generated += 1
        reached = {p.initial_state: 0}

        while not frontier.empty():
            node = frontier.get()[0] # get SearchNode of the tuple
            self.expanded += 1

            for action in p.actions(node.state):
                succ, cost = p.result(node.state, action)
                f = node.g + self.w * p.h(node.state)              # evaluation function f
                succ_node = SearchNode(succ, node, 1)
                # early goal test
                if p.is_goal(succ):
                    return self.extract_path(succ_node), f

                # mark reached to avoid cycles
                if succ not in reached.keys():
                    reached[succ] = f # add successor node in reached with value f
                    # enqueue successor
                    tup = (f, succ_node)
                    frontier.put(tup) # do immer problem :S
                    self.generated += 1

        # no solution found
        return None, None


if __name__ == "__main__":
    problem = pancake_problem.generate_random_problem(5)
    problem = PancakeProblem((1, 5, 6, 2, 4, 3))
    problem.dump()
    astar = WeightedAStarSearch(problem, weight=1, print_statistics=True)
    astar.run()
