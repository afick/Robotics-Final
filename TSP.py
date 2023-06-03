import itertools
import random
import sys
import numpy as np
import queue
import math

RESOLUTION = 1
SLACK = 0
ROBOT = 0

class TSP:

    def held_karp(self, dists):
        """
        Implementation of Held-Karp, an algorithm that solves the Traveling
        Salesman Problem using dynamic programming with memoization.

        Parameters:
            dists: distance matrix

        Returns:
            A tuple, (cost, path).
        """
        n = len(dists)

        # Maps each subset of the nodes to the cost to reach that subset, as well
        # as what node it passed before reaching this subset.
        # Node subsets are represented as set bits.
        C = {}

        # Set transition cost from initial state
        for k in range(1, n):
            C[(1 << k, k)] = (dists[0][k], 0)

        # Iterate subsets of increasing length and store intermediate results
        # in classic dynamic programming manner
        for subset_size in range(2, n):
            for subset in itertools.combinations(range(1, n), subset_size):
                # Set bits for all nodes in this subset
                bits = 0
                for bit in subset:
                    bits |= 1 << bit

                # Find the lowest cost to get to this subset
                for k in subset:
                    prev = bits & ~(1 << k)

                    res = []
                    for m in subset:
                        if m == 0 or m == k:
                            continue
                        res.append((C[(prev, m)][0] + dists[m][k], m))
                    C[(bits, k)] = min(res)

        # We're interested in all bits but the least significant (the start state)
        bits = (2**n - 1) - 1

        # Calculate optimal cost
        res = []
        for k in range(1, n):
            res.append((C[(bits, k)][0] , k))
        opt, parent = min(res)

        # Backtrack to find full path
        path = []
        for _ in range(n - 1):
            path.append(parent)
            new_bits = bits & ~(1 << parent)
            __, parent = C[(bits, parent)]
            bits = new_bits
        
        path.append(0)

        return opt, list(reversed(path))


    def generate_distances(self, n):
        dists = [[0] * n for i in range(n)]
        for i in range(n):
            for j in range(i+1, n):
                dists[i][j] = dists[j][i] = random.randint(1, 99)

        return dists

    def expand_boundaries(self, robot_size, grid):
            '''
            Function that creates binary matrix of explorable cells, and expands size of obstacles
            '''
            rows = len(grid)
            cols = len(grid[0])
            # Initialize a map of the same size that can be fully explored 
            binary_map = [[False for i in range(cols)] for j in range(rows)]
            # Calculate the expansion factor
            expand = int(robot_size // RESOLUTION) + SLACK

            # Iterate through each cell
            for r in range(rows):
                for c in range(cols):
                    # If there is an obstacle in the map
                    if grid[r][c] != 0: 
                        # Mark this cell as not explorable, and expand this by the appropriate factor
                        binary_map[r][c] = True
                        for v in range(1, expand+1):
                            if r-v >= 0:
                                binary_map[r-v][c] = True
                            if r+v < rows:
                                binary_map[r+v][c] = True
                            if c-v >= 0:
                                binary_map[r][c-v] = True
                            if c+v < cols:
                                binary_map[r][c+v] = True
                            
            return np.array(binary_map)

    def validate(self, cell, visited):
        '''
        Function to validate if a cell can be visited
        '''
        row = cell[0]
        col = cell[1]
        rows = len(visited)
        cols = len(visited[0])

        # Check that the cell is in bounds
        if (row < 0 or col < 0 or row >= rows or col >= cols):
            return False

        # If the cell is not explorable
        if (visited[row][col]):
            return False

        # Otherwise, it can be visited
        return True

    def find_bfs_path(self, grid, start, end):
        '''
        Function to find a path from the provided start and end using BFS
        '''
        # 8 Possible row and column movements
        dRow = [-1, 0, 0, 1, 1, 1, -1, -1]
        dCol = [0, 1, -1, 0, 1, -1, -1, 1]

        # Check that the start and goal are reachable
        if self.validate(end, grid) == False:
            exit("goal in obstacle")
        if self.validate(start, grid) == False:
            exit("start in obstacle")

        # Establish queue of nodes we will explore
        frontier = queue.Queue()
        frontier.put(start)

        # Keep track of where a cell came from
        cell_pointers = {}

        grid[start[0]][start[1]] = True

        # Iterate through the possible nodes
        while frontier.empty() == False:
            current = frontier.get()

            if current == end:
                break

            # Add applicable of the 8 surrounding nodes, keep track of cell origins
            for i in range(len(dRow)):
                    next = (current[0] +  dCol[i], current[1] + dRow[i])
                    if self.validate(next, grid):
                        grid[next[0]][next[1]] = True
                        frontier.put(next)
                        cell_pointers[next] = current

        # Backtrack through cell connections to find path
        solution = [] # Stores the path between the nodes
        dist = 0 # Stores the distance between the nodes based on each step to get there
        node = end
        solution.append(node)
        node = cell_pointers[node]
        while node != start:
            step = abs(solution[-1][0] - node[0]) + abs(solution[-1][1] - node[1])
            if step == 2:
                step = math.sqrt(2)
            dist += step
            solution.append(node)
            node = cell_pointers[node]
        solution.append(start)
        step = abs(solution[-2][0] - node[0]) + abs(solution[-2][1] - node[1])
        if step == 2:
            step = math.sqrt(2)
        dist += step
        solution.reverse()

        return dist, solution

    def find_a_star_path(self, grid, start, end):
        '''
        Function to find a path from the provided start and end using A*
        '''
        # 8 Possible row and column movements
        dRow = [-1, 0, 0, 1, 1, 1, -1, -1]
        dCol = [0, 1, -1, 0, 1, -1, -1, 1]

        # Check that the start and goal are reachable
        if self.validate(end, grid) == False:
            exit("goal in obstacle")
        if self.validate(start, grid) == False:
            exit("start in obstacle")

        # Establish priority queue of nodes we will explore
        frontier = queue.PriorityQueue()
        frontier.put((0, start))  # Add start node with priority 0

        # Keep track of where a cell came from
        cell_pointers = {}

        # Keep track of the cost of reaching each node from the start
        costs = {start: 0}

        # Iterate through the possible nodes
        while not frontier.empty():
            _, current = frontier.get()

            if current == end:
                break

            # Add applicable of the 8 surrounding nodes, keep track of cell origins
            for i in range(len(dRow)):
                next = (current[0] + dCol[i], current[1] + dRow[i])
                if self.validate(next, grid):
                    new_cost = costs[current] + self.distance(current, next)
                    if next not in costs or new_cost < costs[next]:
                        costs[next] = new_cost
                        priority = new_cost + self.heuristic(next, end)
                        frontier.put((priority, next))
                        cell_pointers[next] = current

        # Backtrack through cell connections to find path
        solution = []  # Stores the path between the nodes
        dist = costs[end]  # Distance is the cost of reaching the end node
        node = end
        solution.append(node)
        node = cell_pointers[node]
        while node != start:
            solution.append(node)
            node = cell_pointers[node]
        solution.append(start)
        solution.reverse()

        return dist, solution

    def distance(self, pos1, pos2):
        '''
        Function to calculate the distance between two positions using Manhattan distance
        '''
        return math.sqrt(abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1]))

    def heuristic(self, pos, goal):
        '''
        Heuristic function to estimate the cost of reaching the goal from a given position
        In this case, we use the Manhattan distance as the heuristic.
        '''
        return self.distance(pos, goal)

    def calc_distances(self, targets, grid): 
        ''' 
        Function that creates adjacency matrix for the target nodes 
        based on the occupancy grid, using A* to find the shortest distance between each node.
        '''
        targets.insert(0, (0,0)) ## TODO: Replace with current pos.
        # Initialize empty adjacency matrix
        adjacency = np.zeros((len(targets), len(targets)))
        indices = [i for i in range(len(targets))]
        # Create map of reachable and unreachable positions
        binary_map = self.expand_boundaries(ROBOT, grid)
        # Store the paths between two positions
        paths = {}
        print(binary_map)

        # Go through all possible combinations of 2 targets
        for start, end in list(itertools.combinations(indices, 2)):
            map_use = binary_map.copy()
            # Find the bfs path and length between two spots
            length, steps = self.find_a_star_path(map_use, targets[start], targets[end])
            # Store the sub paths and lengths
            paths[(targets[start], targets[end])] = steps
            paths[(targets[end], targets[start])] = steps[::-1]
            adjacency[start][end], adjacency[end][start] = length, length

        return adjacency, paths

    def determine_sequence(self, targets, grid):
        '''
        Function to determine the sequence in which to visit the targets
        '''
        # Assemble the distance adjacency matrix, and store the paths between individual targets
        dists, subpaths = self.calc_distances(targets, grid)

        # Perform held_karp algorithm to minimize total distance
        time, path = self.held_karp(dists)

        # Put together the full path by combining 
        # the sub paths between the individual targets
        total_path = []
        for i in range(len(path) - 1):
            start = path[i]
            end = path[i+1]
            total_path.append(subpaths[(targets[start], targets[end])])

        return time, path, total_path

    def read_distances(self, filename):
        '''
        Function to read adjacency graph from a csv
        '''
        dists = []
        with open(filename, 'rb') as f:
            for line in f:
                # Skip comments
                if line[0] == '#':
                    continue
                line = list(map(int, line.decode().strip().split(',')))
                dists.append(line)
        return dists


    def unit_test(self):
        # make all of the -1's 100s before passing map into determine sequence
        grid = [[0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 100, 0, 0, 0],
                [0, 0, 100, 100, 100, 0, 0],
                [0, 0, 0, 100, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0]]
        print(grid)
        print(self.determine_sequence([(5,5), (6,6), (1,1)], grid))

    if __name__ == '__main__':

        unit_test()

        # arg = sys.argv[1]


        # if arg.endswith('.csv'):
        #     dists = read_distances(arg)
        # else:
        #     dists = generate_distances(int(arg))
        #
        # # Pretty-print the distance matrix
        # for row in dists:
        #     print(''.join([str(n).rjust(4, ' ') for n in row]))

        # print('')

        # print(held_karp(dists))