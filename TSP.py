import itertools
import random
import sys
import numpy as np
import Queue


def held_karp(dists):
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

    return opt, list(reversed(path))


def generate_distances(n):
    dists = [[0] * n for i in range(n)]
    for i in range(n):
        for j in range(i+1, n):
            dists[i][j] = dists[j][i] = random.randint(1, 99)

    return dists

def expand_boundaries(robot_size, grid):
        '''
        Function that creates binary matrix of explorable cells, and expands size of obstacles
        '''
        rows = len(grid)
        cols = len(grid[0])
        # Initialize a map of the same size that can be fully explored 
        binary_map = [[False for i in range(cols)] for j in range(rows)]
        # Calculate the expansion factor
        expand = int(ROBOT_SIZE // self.resolution) + SLACK

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

def validate(cell, visited):
    '''
    Function to validate if a cell can be visited
    '''
    row = cell[1]
    col = cell[0]
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

def find_bfs_path(grid, start, end):
        '''
        Function to find a path from the provided start and end using BFS
        '''
        # 8 Possible row and column movements
        dRow = [-1, 0, 0, 1, 1, 1, -1, -1]
        dCol = [0, 1, -1, 0, 1, -1, -1, 1]

        # Check that the start and goal are reachable
        if validate(end, grid) == False:
            exit("goal in obstacle")
        if validate(start, grid) == False:
            exit("start in obstacle")

        # Establish queue of nodes we will explore
        frontier = Queue.Queue()
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
                    if validate(next, grid):
                        grid[next[0]][next[1]] = True
                        frontier.put(next)
                        cell_pointers[next] = current

        # Backtrack through cell connections to find path
        solution = [] 
        node = end
        while node != start:
            solution.append(node)
            node = cell_pointers[node]
        solution.append(start)
        solution.reverse()
        return solution

def calc_distances(targets, grid): 
    ''' 
    Function that creates adjacency matrix for the target nodes 
    based on the occupancy grid, using A* to find the shortest distance between each node.
    '''
    adjacency = np.zeros((len(targets) +1, len(targets) +1))
    indices = [i for i in range(len(targets))]
    binary_map = expand_boundaries(grid)
    for start, end in list(itertools.combinations(indices, 2)):
        map_use = binary_map.copy()
        length = find_bfs_path(map_use, targets[start], targets[end])
        adjacency[start][end], adjacency[end][start] = length, length
    return adjacency

def determine_sequence(targets, grid):
    dists = calc_distances(targets, grid)

    time, path = held_karp(dists)
    return path

def read_distances(filename):
    dists = []
    with open(filename, 'rb') as f:
        for line in f:
            # Skip comments
            if line[0] == '#':
                continue
            line = list(map(int, line.decode().strip().split(',')))
            dists.append(line)
    return dists


if __name__ == '__main__':
    arg = sys.argv[1]

    if arg.endswith('.csv'):
        dists = read_distances(arg)
    else:
        dists = generate_distances(int(arg))

    # Pretty-print the distance matrix
    for row in dists:
        print(''.join([str(n).rjust(4, ' ') for n in row]))

    print('')

    print(held_karp(dists))