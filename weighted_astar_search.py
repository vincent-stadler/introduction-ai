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
    """
    """

if __name__ == "__main__":
  problem = pancake_problem.generate_random_problem(5)
  problem = PancakeProblem((1, 5, 6, 2, 4, 3))
  problem.dump()
  astar = WeightedAStarSearch(problem, 1, print_statistics=True)
  astar.run()



