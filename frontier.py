# Author: Carter Kruse
# Date: May 18, 2023

# Import Relevant Libraries (Python Modules)
from numpy import genfromtxt
from matplotlib import pyplot as plt

class frontier:

    def frontier_exploration(self, map):

        """
        Input: 2D array form of the map
        Output: List of frontier points to be visited
        """

        # Region-Growing (BFS)
        regions = []
        to_visit = set()

        # Add to the 'to_visit' set.
        for i in range(len(map)):
            for j in range(len(map[0])):
                if map[i][j] != 0:
                    to_visit.add((j, i))
        
        # Algorithm
        while (len(to_visit) != 0):
            queue = []
            region = []

            start_point = to_visit.pop()
            queue.append(start_point)

            neighbors_offsets = [(0, 1), (0, -1), (1, 0), (-1, 0)]

            # Continue to loop through points in the queue (until it is empty).
            while (len(queue) != 0):
                current_point = queue.pop(0)
                region.append(current_point)

                # Neighboring Points
                for offset in neighbors_offsets:
                    neighbor_point = (current_point[0] + offset[0], current_point[1] + offset[1])

                    # Check the threshold condition (in the points to visit).
                    if neighbor_point in to_visit:
                        # Add the point to the queue and remove it from 'to_visit'.
                        queue.append(neighbor_point)
                        to_visit.remove(neighbor_point)
            
            # Minimum Region Size
            if len(region) > 5:
                regions.append(region)
        
        # print(regions)
        # print(len(regions))

        # # # # #

        points = []

        for region in regions:
            points.append([sum(ele) / len(region) for ele in zip(*region)])
        
        print(points)

        return points


#####################################################################


def main():
    # Read the data from the CSV file.
    data = genfromtxt('data.csv', delimiter = ',')

    boundary_data = data.copy()

    for i in range(1, len(data) - 1):
        for j in range(1, len(data[0]) - 1):
            p1 = abs(data[i][j] - data[i][j - 1])
            p2 = abs(data[i][j] - data[i][j + 1])
            p3 = abs(data[i][j] - data[i - 1][j])
            p4 = abs(data[i][j] - data[i + 1][j])

            boundary_data[i][j] = max(p1, p2, p3, p4)

    # # # # #

    # Update the appropriate data.
    for i in range(len(data)):
        for j in range(len(data[0])):
            if data[i][j] == -1:
                data[i][j] = -100
    
    # # # # #

    updated_boundary_data = data.copy()

    for i in range(1, len(data) - 1):
        for j in range(1, len(data[0]) - 1):
            p1 = abs(data[i][j] - data[i][j - 1])
            p2 = abs(data[i][j] - data[i][j + 1])
            p3 = abs(data[i][j] - data[i - 1][j])
            p4 = abs(data[i][j] - data[i + 1][j])

            updated_boundary_data[i][j] = max(p1, p2, p3, p4)
    
    difference_data = data.copy()

    for i in range(1, len(data) - 1):
        for j in range(1, len(data[0]) - 1):
            difference_data[i][j] = boundary_data[i][j] - updated_boundary_data[i][j]
    
    inter_data = data.copy()

    for i in range(1, len(data) - 1):
        for j in range(1, len(data[0]) - 1):
            inter_data[i][j] = difference_data[i][j] - updated_boundary_data[i][j]
    
    final_data = inter_data.copy()

    for i in range(len(inter_data)):
        for j in range(len(inter_data[0])):
            if abs(inter_data[i][j]) >= 230 or abs(inter_data[i][j]) <= 150:
                final_data[i][j] = 0
    
    # # # # #

    open_space = data.copy()

    # Update the appropriate data.
    for i in range(len(data)):
        for j in range(len(data[0])):
            if abs(data[i][j]) >= 10:
                open_space[i][j] = 0
            else:
                open_space[i][j] = 200
    
    # # # # #

    # Display the grid/map
    # plt.figure(figsize = (100, 100))
    # image = plt.imshow(final_data, origin = 'lower')

    # plt.colorbar(image)
    # plt.show()

    # # # # #

    # Region-Growing (BFS)
    regions = []
    to_visit = set()

    # Add to the 'to_visit' set.
    for i in range(len(final_data)):
        for j in range(len(final_data[0])):
            if final_data[i][j] != 0:
                to_visit.add((j, i))
    
    # Algorithm
    while (len(to_visit) != 0):
        queue = []
        region = []

        start_point = to_visit.pop()
        queue.append(start_point)

        neighbors_offsets = [(0, 1), (0, -1), (1, 0), (-1, 0)]

        # Continue to loop through points in the queue (until it is empty).
        while (len(queue) != 0):
            current_point = queue.pop(0)
            region.append(current_point)

            # Neighboring Points
            for offset in neighbors_offsets:
                neighbor_point = (current_point[0] + offset[0], current_point[1] + offset[1])

                # Check the threshold condition (in the points to visit).
                if neighbor_point in to_visit:
                    # Add the point to the queue and remove it from 'to_visit'.
                    queue.append(neighbor_point)
                    to_visit.remove(neighbor_point)
        
        # Minimum Region Size
        if len(region) > 5:
            regions.append(region)
    
    # print(regions)
    # print(len(regions))

    # # # # #

    points = []

    for region in regions:
        points.append([sum(ele) / len(region) for ele in zip(*region)])
    
    print(points)

    # # # # #

    # MatPlotLib
    colors = ['red', 'green', 'purple', 'orange', 'yellow']

    fig, axs = plt.subplots(2, 3)
    implot = axs[0, 0].imshow(data, origin = 'lower')
    implot = axs[0, 1].imshow(boundary_data, origin = 'lower')
    implot = axs[0, 2].imshow(updated_boundary_data, origin = 'lower')
    implot = axs[1, 0].imshow(difference_data, origin = 'lower')
    implot = axs[1, 1].imshow(inter_data, origin = 'lower')
    implot = axs[1, 2].imshow(final_data, origin = 'lower')

    for k in range(len(points)):
        for i in range(2):
            for j in range(3):
                axs[i, j].scatter(points[k][0], points[k][1], s = 80, c = colors[k % 5], marker = 'x')

    plt.show()

if __name__ == "__main__":
    """Run the main function."""
    main()
