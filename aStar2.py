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
    # print(node.coord[0], node.coord[1])
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
            dg += 1.5
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
        print(f"c: {best_node.coord}, h: {heuristic(best_node, self.goal)}, g: {self.g_score(best_node)}")
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
    
    for i in range(rows):
        for j in range(cols):
            if grid[i, j] == 0:
                # Check all adjacent cells and set them to 0 if they are 1
                for x in range(max(0, i-1), min(rows, i+2)):
                    for y in range(max(0, j-1), min(cols, j+2)):
                        if grid[x, y] == 1:
                            transformed_grid[x, y] = 0
                            
    return transformed_grid

def visualize_path(grid, path, xy_path):
    for coord in path:
        grid[coord[0], coord[1]] = 2  # Mark the path with 2

    print(grid)
    plt.imshow(grid, cmap='viridis')
    plt.show()
    plt.scatter(xy_path[:, 0], xy_path[:, 1])
    plt.show()

def convert_to_meters(grid, path, dim_meters):
    x_node_len = (dim_meters[0]/grid.shape[1])/2
    y_node_len = (dim_meters[1]/grid.shape[0])/2

    path = np.array(path)
    path[:, [1, 0]] = path[:, [0, 1]]
    path = (np.array([1,-1]) * path) + np.array([1, grid.shape[0]])
    path = (path * 2 * np.array([x_node_len, y_node_len])) - np.array([x_node_len, y_node_len])
    return path
    

if __name__ == "__main__":
    # start and end positions
    start_point = [0, 0]
    end_point = [11, 11]
    # end_point = np.array([10, 6])

    x_meters = 2
    y_meters = 1

    dim_meters = np.array([x_meters, y_meters])

    A = aStar()
    A.goal = np.array(end_point)
    A.initial_node(start_point)

    A.grid_obs = np.array([
        [1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0],
        [1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0],
        [0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1],
        [1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1],
        [1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    ])

    A.grid_adj = transform_adjacent_ones_to_zeros(A.grid_obs)

    while not A.run_one_step():
        A.run_one_step()

    path = A.get_path()
    # print("Optimal Path:", path)
    xy_path = convert_to_meters(A.grid_obs, path, dim_meters)
    print("XY Optimal Path in meters: \n", xy_path)
    visualize_path(A.grid_obs, path, xy_path)



    
    