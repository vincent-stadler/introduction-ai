# Stadler Ly Hofmann
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
        frontier = PriorityQueue()
        first_tup = (0, SearchNode(p.initial_state, None,
                                   0))  # (cost, node (comparison between nodes is implemented in the SearchNode class, LIFO))
        frontier.put(first_tup)  # cost = heurestics of node.state and node.g
        self.generated += 1
        reached = {p.initial_state: 0}

        while not frontier.empty():
            node = frontier.get()[-1]
            self.expanded += 1

            for action in p.actions(node.state):
                succ, cost = p.result(node.state, action)
                new_g = node.g + cost
                succ_node = SearchNode(succ, node, new_g)
                # early goal test
                if p.is_goal(succ):
                    return self.extract_path(succ_node), new_g

                # mark reached to avoid cycles
                if succ not in reached.keys() or new_g < reached[succ]:
                    # here the heuristic of the succ node and it's path cost are calculated
                    # print('path cost', new_g, 'heuristics', p.h(succ) )
                    heuristics_and_path_cost = new_g + self.w * p.h(succ)
                    reached[succ] = new_g
                    # enqueue successor
                    frontier.put(tuple((heuristics_and_path_cost, succ_node)))
                    self.generated += 1


if __name__ == "__main__":
    problem = pancake_problem.generate_random_problem(5)
    problem = PancakeProblem((1, 5, 6, 2, 4, 3))
    problem.dump()
    astar = WeightedAStarSearch(problem, 1, print_statistics=True)
    astar.run()
