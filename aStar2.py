import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import NDArray
from typing import List, Union
from dataclasses import dataclass
import time



@dataclass
class Node:
    parent: Union["Node", None] = None
    coord: NDArray = np.array([0,0])
    g: float = 0
    h: float = 0
    f: float = 0
    children: Union[List["Node"], None] = None

def heuristic(node: Node, goal: NDArray) -> float:
    distVect = node.coord - goal
    hScore = 2*np.linalg.norm(distVect)
    return hScore

def is_on_zero(node: Node, grid: NDArray) -> bool:
    if grid[node.coord[0], node.coord[1]] == 0:
        return True
    return False

class aStar:
    def __init__(self):
        self.nodeArr: List[Node] = []
        self.grid_obs: NDArray = np.empty(0, dtype=int)
        self.grid_adj: NDArray = np.empty(0, dtype=int)
        self.goal: NDArray = np.empty(0, dtype=int)

    def g_score(self, node):
        if node.parent is None:
            return 0
        
        dg = np.linalg.norm(node.coord - node.parent.coord)
        if is_on_zero(node, self.grid_adj):
            dg += 1
        return (node.parent.g + dg)
    
    def h_score(self, node):
        return(heuristic(node, self.goal))

    def f_score(self, node):
        return self.h_score(node) + self.g_score(node)

    def is_valid(self, node: Node):
        s = self.grid_obs.shape
        minX = 0
        minY = 0
        maxX = s[0]
        maxY = s[1]
        if 0 <= node.coord[0] < maxX and 0 <= node.coord[1] < maxY:
            return True
        else:
            return False

    def get_neighbors(self, node: Node):
        neighborsCoord = [
            (node.coord[0] + 1, node.coord[1]),
            (node.coord[0] - 1, node.coord[1]),
            (node.coord[0], node.coord[1] + 1),
            (node.coord[0], node.coord[1] - 1),
            (node.coord[0] + 1, node.coord[1] + 1),
            (node.coord[0] + 1, node.coord[1] - 1),
            (node.coord[0] - 1, node.coord[1] + 1),
            (node.coord[0] - 1, node.coord[1] - 1)
        ]

        neighList: List[Node] = []
        for c in neighborsCoord:
            newNode = Node()
            newNode.parent = node
            newNode.coord = np.array(c)
            neighList.append(newNode)

        valid_neighbor_list: List[Node] = []
        for n in neighList:
            if not self.is_valid(n):
                valid_neighbor = False
                continue
            if is_on_zero(n, self.grid_obs):
                valid_neighbor = False
            else:
                valid_neighbor = True
            if valid_neighbor:
                valid_neighbor_list.append(n)
        return(valid_neighbor_list)

    def fill_score(self, node):
        node.h = self.h_score(node)
        node.g = self.g_score(node)
        node.f = self.f_score(node)

    def find_lowest_f(self):
        min_f = np.inf
        best_node = None
        for n in self.nodeArr:
            not_explored = n.children is None
            if not not_explored:
                continue # skips
            if n.f < min_f:
                min_f = n.f
                best_node = n
        return best_node

    def explore(self, node):
        neighbors: List[Node] = self.get_neighbors(node)
        node.children = neighbors
        for n in neighbors:
            self.fill_score(n)
        self.nodeArr += neighbors
    
    def run_one_step(self):
        best_node = self.find_lowest_f()
        # print(f"c: {best_node.coord}, h: {heuristic(best_node, self.goal)}, g: {self.g_score(best_node)}")
        if heuristic(best_node, self.goal) < 1:
            self.goal_node = best_node
            return True
        self.explore(best_node)
        return False

    def initial_node(self, start_point):
        start = Node()
        start.coord = np.array(start_point)
        start.parent = None
        self.fill_score(start)
        self.nodeArr.append(start)

    def get_path(self):
        path = []
        node = self.goal_node
        while node is not None:
            path.append(node.coord)
            node = node.parent
        return path[::-1]

def transform_adjacent_ones_to_zeros(grid):
    # Create a copy of the grid to avoid modifying the original array
    transformed_grid = grid.copy()
    rows, cols = grid.shape

    # Transform edge 1s to 0s
    for i in range(rows):
        if grid[i, 0] == 1:  # Left edge
            transformed_grid[i, 0] = 0
        if grid[i, cols - 1] == 1:  # Right edge
            transformed_grid[i, cols - 1] = 0

    for j in range(cols):
        if grid[0, j] == 1:  # Top edge
            transformed_grid[0, j] = 0
        if grid[rows - 1, j] == 1:  # Bottom edge
            transformed_grid[rows - 1, j] = 0

    # Transform adjacent 1s to 0s
    for i in range(rows):
        for j in range(cols):
            if grid[i, j] == 0:
                # Check all adjacent cells and set them to 0 if they are 1
                for x in range(max(0, i-1), min(rows, i+2)):
                    for y in range(max(0, j-1), min(cols, j+2)):
                        if grid[x, y] == 1:
                            transformed_grid[x, y] = 0

    return transformed_grid

def visualize_path(grid, path, xy_path, dim_meters):
    for coord in path:
        grid[coord[0], coord[1]] = 2  # Mark the path with 2

    print(grid)

    fig, ax1 = plt.subplots()
    im = ax1.imshow(grid, cmap='viridis')

    # Set custom tick labels for x and y axes
    ax1.set_xticks(np.linspace(0, grid.shape[1] - 1, 5))  # Customize the number of ticks
    ax1.set_xticklabels(np.linspace(0, dim_meters[0], 5))

    ax1.set_yticks(np.linspace(0, grid.shape[0] - 1, 5))
    ax1.set_yticklabels(np.linspace(dim_meters[1], 0, 5))  # Reverse the tick labels to start from 0 at the bottom

    plt.show()

def convert_to_meters(grid, path, dim_meters):
    x_node_len = (dim_meters[0]/grid.shape[1])/2
    y_node_len = (dim_meters[1]/grid.shape[0])/2

    path = np.array(path)
    path[:, [1, 0]] = path[:, [0, 1]]
    path = (np.array([1,-1]) * path) + np.array([1, grid.shape[0]])
    path = (path * 2 * np.array([x_node_len, y_node_len])) - np.array([x_node_len, y_node_len])
    return path

def convert_to_cells(grid, obs_pos, dim_meters):
    x_node_len = (dim_meters[0]/grid.shape[1])/2
    y_node_len = (dim_meters[1]/grid.shape[0])/2
    
    i = 0
    for obs in obs_pos:
        x1, y1 = obs[0]
        x2, y2 = obs[1]

        x1 = int(np.floor((x1/dim_meters[0])*grid.shape[1]))
        y1 = int(grid.shape[0] - np.ceil((y1/dim_meters[1])*grid.shape[0]))

        x2 = int(np.ceil((x2/dim_meters[0])*grid.shape[1]))
        y2 = int(grid.shape[0] - np.floor((y2/dim_meters[1])*grid.shape[0]))

        print(x1, y1)

        obs_pos[i] = ((y1, x1), (y2, x2))
        i += 1
    # print(obs_pos)
    return obs_pos

def replace_square_with_zeros(grid, obs_nodes):
    for obs in obs_nodes:
        row1, col1 = obs[0]
        row2, col2 = obs[1]

        # Determine the bounds of the square
        top_row = min(row1, row2)
        bottom_row = max(row1, row2)
        left_col = min(col1, col2)
        right_col = max(col1, col2)
    
        # Replace the square with 0's
        grid[top_row:bottom_row+1, left_col:right_col+1] = 0
    return grid

def start_end_cells(start_point, end_point, dim_meters, grid):
    x1 = np.clip(round((start_point[0] / dim_meters[0]) * grid.shape[1]), 0, grid.shape[1] - 1)
    y1 = np.clip(round(grid.shape[0] - 1 - np.ceil((start_point[1] / dim_meters[1]) * grid.shape[0])), 0, grid.shape[0] - 1)
    start_cell = np.array([y1, x1])

    x2 = np.clip(round((end_point[0] / dim_meters[0]) * grid.shape[1]), 0, grid.shape[1] - 1)
    y2 = np.clip(round(grid.shape[0] - 1 - np.ceil((end_point[1] / dim_meters[1]) * grid.shape[0])), 0, grid.shape[0] - 1)
    end_cell = np.array([y2, x2])

    return start_cell, end_cell

def run(start_point, end_point, x_Max, y_Max, obs_pos):
    dim_meters = np.array([x_Max, y_Max])

    # convert x_max and y_max to grid size
    x_Nodes = int(np.floor(x_Max/0.1)) 
    y_Nodes = int(np.floor(y_Max/0.1))
    grid_init = np.ones((y_Nodes, x_Nodes))

    start_cell, end_cell = start_end_cells(start_point, end_point, dim_meters, grid_init)

    print(start_cell, end_cell)

    A = aStar()
    A.goal = np.array(end_cell)
    A.initial_node(start_cell)

    obs_nodes = convert_to_cells(grid_init, obs_pos, dim_meters)
    print(obs_nodes)

    A.grid_obs = replace_square_with_zeros(grid_init, obs_nodes)
    A.grid_adj = transform_adjacent_ones_to_zeros(A.grid_obs)

    print(A.grid_obs)

    while not A.run_one_step():
        A.run_one_step()

    path = A.get_path()
    xy_path = convert_to_meters(A.grid_obs, path, dim_meters)
    print("XY Optimal Path in meters: \n", xy_path)
    visualize_path(A.grid_obs, path, xy_path, dim_meters)
    

if __name__ == "__main__":
    # inputs:
    start_point = [0, 0] # xy meters
    end_point = [2.0, 1.0] # xy meters

    x_Max = 2.06 # meters
    y_Max = 1.00 # meters

    obs_pos = [((0.5, 0.4), (0.3, 0.3)),
                ((1.3, 0.7), (1.5, 0.5)),
                ((0.6, 1.0), (0.7, 0.7)),
                ((1.3, 0.25), (1.5, 0.2))]

    #run code using inputs:
    run(start_point, end_point, x_Max, y_Max, obs_pos)
