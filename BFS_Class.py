class BFS:
    """BFS Class - To understand the content of the class, please read the docstrings carefully."""

    def bfs(map, start_point, end_point):
        """BFS Function"""
        """When the method / function is called, it will return the appropriate path of the robot corresponding
        to a given map (occupancy grid), along with the starting and ending points. This is where the actual BFS
        algorithm occurs.
        """

        """Parameters"""
        """The map (as an occupancy grid) is given as 'map', the starting point is given as 'start_point', and
        the ending point is given as 'end_point'."""

        """Calculations - BFS Algorithm"""
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
