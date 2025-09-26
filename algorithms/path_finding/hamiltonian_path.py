import heapq
import logging
import multiprocessing as mp
import time
from typing import List
import math
from enum import Enum

from arena.map import Map
from common.types import Position
from common.utils import euclidean
from path_finding.astar import AStar, Node


MAX_ASTAR_F_COST = 99999

logger = logging.getLogger('HAMILTONIAN PATH')

# `knn()` -> Not Used
def knn(mp: "Map", src: "Position") -> List[List["Node"]]:
    astar = AStar(mp)
    path = [src] + [o.to_pos() for o in mp.obstacles]

    for i in range(1, len(path)):
        mn = float('inf')
        mn_i = i+1

        for j in range(i, len(path)):
            dist = euclidean(path[i-1], path[j])
            if dist < mn:
                mn = dist
                mn_i = j

        path[mn_i], path[i] = path[i], path[mn_i]

    res = []
    prev = src
    for i in range(1, len(path)):
        res.append(astar.search(prev, path[i]))
        prev = res[-1][-1].c_pos
    return res


def _permutate(n: int, start_from_zero: bool) -> List[List[int]]:
    """
    This function generates all permutations of numbers from 0 to n-1 and returns them as a list of lists. 
    If `start_from_zero` is True, it filters the permutations to include only those where 0 is the first element.

    Example:
        If n = 4, the function will return the permutations of [[0, 1, 2, 3]] or [[1, 2, 3, 0]] depending on the value of start_from_zero.

    """
    res = []

    def helper(curr: List[int]):
        if len(curr) == n:
            res.append(curr)
            return
        for i in range(n):
            if i not in curr:
                helper([*curr, i])
    helper([])
    if start_from_zero:
        res = list(filter(lambda p:p[0] == 0, res))
    return res


class AlgoType(Enum):
    """Enumeration for possible algorithms to be used for `HamiltonianSearch`"""
    EXHAUSTIVE_ASTAR = "Exhaustive Astar"
    EUCLIDEAN = "Euclidean"
    BFS = "Breadth First Search"

class SearchProcess(mp.Process):
    """A Process (similar to a Thread) used for multiprocessing to speed up algorithm computation time"""
    def __init__(
        self,
        pos: List["Position"],
        astar: AStar,
        todo: mp.Queue,
        done: mp.Queue,
        i:int,
        algo_type = AlgoType
    ):
        super().__init__()
        self.astar = astar
        self.pos = pos
        self.todo = todo
        self.done = done
        self.i = i
        self.algo_type = algo_type
        logger.info(f'Spawning P{i}')


    def _search(
        self,
        st: int,
        end: int
    ) -> float:
        """Search According to the `AlgoType`"""
        logger.info(f'P{self.i} start search {st, end}')

        # Ensure algo_type is always an AlgoType Enum
        if not isinstance(self.algo_type, AlgoType):
            algo_type = AlgoType(self.algo_type)
        else:
            algo_type = self.algo_type

        match (algo_type):
            case AlgoType.EXHAUSTIVE_ASTAR:
                path = self.astar.search(self.pos[st], self.pos[end])
                return path[-1].f if path else MAX_ASTAR_F_COST
            case AlgoType.EUCLIDEAN:
                start_pos = self.pos[st]
                end_pos = self.pos[end]
                euclidean_distance = math.sqrt((end_pos.x - start_pos.x) ** 2 + (end_pos.y - start_pos.y) ** 2)
                return euclidean_distance
            case AlgoType.BFS:
                # TODO: BFS
                raise NotImplementedError()
            case _:
                raise Exception("Invalid AlgoType")
    
    def run(self):
        while not self.todo.empty():
            try:
                r, c = self.todo.get_nowait()
            except:
                break
            f = self._search(r, c)
            self.done.put((r, c, f))

class HamiltonianSearch:
    """
    Uses `Astar` (If AlgoType.EXHAUSIVE_ASTAR) to do an exhaustive search on all possible permutations of order of obstacles to visit 
    and finds the lowest cost permutation and its associated paths.

    Uses Multiprocessing (parameter `n` which defaults to 8) to lower computation time.

    Params:

        `map`: Map object
        `src`: Position object of the source/starting position
        `n` = 8: Number of child processes to run concurrently

    Main Method: `search()`
        
        Returns:
            min_perm`: lowest cost order of visiting all the obstacles starting from starting location;
            `loc_mn_path`: An array of a path Array of `Node` where each inner path Array is the path from one location to another;
    """

    def __init__(
        self,
        map: "Map", 
        src: "Position",
        algo_type: AlgoType,
        n: int = 8
    ):
        self.astar = AStar(map)
        self.src = src
        self.pos = [src] + [o.to_pos() for o in map.obstacles]
        self.n = n
        self.algo_type = algo_type

        # TODO: BFS
        if algo_type == AlgoType.BFS:
            raise NotImplementedError()

    
    def search(self, top_n: int = 3):
        print("----- Start Hamiltonian Search -----")

        st = time.time()
        n = len(self.pos)
        if n == 2:
            # Only start and one obstacle
            segment = self.astar.search(self.pos[0], self.pos[1])
            if segment:
                return [0, 1], [segment]
            else:
                return [], []

        m = int(n*n - n - (n-1)) # total paths to calc from pt to pt (excluding from pt_a to pt_a and from pt_a to 0)
        perms = _permutate(n, True)
        edges = [[0 for _ in range(n)] for _ in range(n)]
        todo = mp.Queue() # (r, c)
        done = mp.Queue() # (r, c, astar f cost)

        # Fill todo queue with all (r, c) pairs except r == c or c == 0
        for r in range(n):
            for c in range(n):
                if r != c and c != 0:
                    todo.put((r, c))

        # Create multiple Processes and start them
        processes = []
        for i in range(self.n):
            p = SearchProcess(self.pos, self.astar, todo, done, i, self.algo_type)
            p.daemon = True
            p.start()
            processes.append(p)

        # Collect results from done queue
        while m:
            r, c, f = done.get()
            edges[r][c] = f
            m -= 1

        # Optionally, terminate processes
        for p in processes:
            p.terminate()

        # get shortest path, i.e., lowest cost among all permutations
        h = []
        for perm in perms:
            f = 0
            for i in range(1, n):
                f += edges[perm[i-1]][perm[i]]
            h.append((f, perm))

        h.sort()
        min_perm = []
        loc_mn_path = []
        for i in range(min(top_n, len(h))):
            f, perm = h[i]
            if f < MAX_ASTAR_F_COST:
                min_perm = perm
                # Reconstruct the actual paths using astar
                prev = self.pos[perm[0]]
                path_segments = []
                for idx in range(1, n):
                    segment = self.astar.search(prev, self.pos[perm[idx]])
                    path_segments.append(segment)
                    prev = self.pos[perm[idx]]
                loc_mn_path = path_segments
                break

        print(f'Time (pathfinding) {time.time()-st} s')
        return min_perm, loc_mn_path