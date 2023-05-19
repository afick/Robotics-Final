# Author: Carter Kruse
# Date: May 18, 2023

# Import Relevant Libraries (Python Modules)
from numpy import genfromtxt
from matplotlib import pyplot as plt

def bfs(map, start_point, end_point):
    visited = {end_point: None}
    queue = [end_point]

    while queue:
        current = queue.pop(0)

        if current == start_point:
            shortest_path = []

            while current:
                shortest_path.append(current)
                current = visited[current]
            
            return shortest_path
        
        adjacent_points =[]

        current_col, current_row = current

        # Up
        if current_row > 0:
            if map[current_row - 1][current_col] != 100:
                adjacent_points.append((current_col, current_row - 1))

        # Right
        if current_col < (len(map[0]) - 1):
            if map[current_row][current_col + 1] != 100:
                adjacent_points.append((current_col + 1, current_row))
        
        # Down
        if current_row < (len(map) - 1):
            if map[current_row + 1][current_col] != 100:
                adjacent_points.append((current_col, current_row + 1))
        
        # Left
        if current_col > 0:
            if map[current_row][current_col - 1] != 100:
                adjacent_points.append((current_col - 1, current_row))
        
        for point in adjacent_points:
            if point not in visited:
                visited[point] = current
                queue.append(point)

def main():
    # Read the data from the CSV file.
    data = genfromtxt('data.csv', delimiter = ',')
    
    # The frontier points determined from the previous script.
    points = [[269.00704225352115, 176.7887323943662], [313.74725274725273, 186.74725274725276], [197.0, 298.6384615384615], [344.2432432432432, 222.35135135135135]]
    
    # Cast to integer values.
    points = [[round(i), round(j)] for [i, j] in points]
    
    # print(points)
    
    # Create adjacency matrix.
    adjacency_matrix = [[len(bfs(data, (points[i][0], points[i][1]), (points[j][0], points[j][1]))) - 1 for j in range(len(points))] for i in range(len(points))]

    print(adjacency_matrix)

if __name__ == "__main__":
    """Run the main function."""
    main()