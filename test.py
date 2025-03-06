from MSTC_Star.mcpp.mstc_star_planner import MSTCStarPlanner
import networkx as nx
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import cv2
from copy import deepcopy
import cv2
import numpy as np
import matplotlib.pyplot as plt
from skimage.morphology import skeletonize
from typing import Tuple, List
from MSTC_Star.utils.nx_graph import mst

class PathCoveragePlanner:

    def __init__(self, grid_map: np.ndarray, robot_positions: List[Tuple[int, int]], map_config: dict) -> None:
        """
        grid_map: grid map as opencv image
        robot_position: starting position in grid map
        map_config: p^{real} = k * R^{real}_{grid} @ p^{grid} + p^{real}_{grid}
                    map_config = {
                        "ratio": 0.0,
                        "R^r_g": np.eye(3),
                        "p^r_g": np.zeros([3, 1]) 
                    }
        debug: show some figures if True
        TODO: dilate the grid map to prevent collision
        """
        # set attributes
        self._grid_map = deepcopy(grid_map)
        self._robot_positions = deepcopy(robot_positions)
        self._map_config = deepcopy(map_config)

        # convert the original grid map to graph
        self._grid_map_graph = self._grid_to_graph(self._grid_map)

        # get the skeleton of the original grid map
        self._skeleton_map = self._extract_skeleton(debug)

        # add the robot positions to the skeleton
        for position in self._robot_positions:
            self._add_robot_to_skeleton(position)
        
        # convert the skeleton to graph for MSTC
        self._skeleton_graph = self._grid_to_graph(self._skeleton_map)

        # give the final result
        self._result_paths, _ = self._mstc_star(self._skeleton_graph, np.inf, is_show=True)

    def _extract_skeleton(self, show: bool = False):
        """
        grid_map: gray scale graid map loaded by opencv
        show: whether show the images
        """

        # Convert to binary (Assuming white=path, black=wall)
        _, thr = cv2.threshold(self._grid_map, 127, 255, cv2.THRESH_BINARY)

        # open & close
        kernel = np.array([[1, 1, 1], [1, 1, 1], [1, 1, 1]])
        thr = cv2.morphologyEx(thr, cv2.MORPH_OPEN, kernel, iterations=1)
        thr = cv2.morphologyEx(thr, cv2.MORPH_CLOSE, kernel, iterations=1)

        # Normalize to 0-1 for skimage
        binary = np.zeros_like(thr)
        binary[thr == 255] = 1

        # Apply skeletonization
        skeleton = skeletonize(binary)
        
        return skeleton
    
    def _add_robot_to_skeleton(self, robot_position: Tuple[int, int]):
        
        # make sure the robot position is available
        assert robot_position in self._grid_map_graph.nodes

        # max range to search for the nearest starting point
        max_range = 25
        # construct searching area
        searching_range = []
        height, width = self._grid_map.shape
        for i in range(-max_range, max_range):
            for j in range(-max_range, max_range):
                x, y = robot_position[0] + i, robot_position[1] + j
                if 0 <= x <= height and 0 <= y <= width and \
                   self._skeleton_map[x, y]:
                    searching_range.append((robot_position[0] + i, robot_position[1] + j))

        min_distance = 1e10
        shortest_path = None
        for (n_x, n_y) in searching_range:
            if not nx.has_path(self._grid_map_graph, source=robot_position, target=(n_x, n_y)):
                continue
            path = nx.shortest_path(self._grid_map_graph, source=robot_position, target=(n_x, n_y), weight="weight")
            length = nx.shortest_path_length(self._grid_map_graph, source=robot_position, target=(n_x, n_y), weight="weight")
            if min_distance > length:
                min_distance = length
                shortest_path = path
        
        assert shortest_path is not None

        for (x, y) in shortest_path:
            self._skeleton_map[x, y] = True

    def _grid_to_graph(self, grid_map: np.ndarray, thresh: int = 128) -> nx.Graph:
        """
        grid_map: gray sacle image or bool type ndarray
        """
        height, width = grid_map.shape
        dtype = grid_map.dtype

        # Create an empty graph
        G = nx.Graph()

        pixel_condition = (lambda x: x) if dtype == bool else (lambda x: x >= thresh)

        # Add nodes for each pixel
        for x in range(height):
            for y in range(width):
                if pixel_condition(grid_map[x, y]):
                    G.add_node((x, y))  # Store intensity as node attribute

        # Define 8-neighbor connectivity
        neighbors = [(-1, -1), (-1, 0), (-1, 1), 
                    (0, -1), (0, 1), 
                    (1, -1), (1, 0), (1, 1)]

        # Add edges between adjacent pixels
        for (x, y) in G.nodes:
            for dx, dy in neighbors:
                n_x, n_y = x + dx, y + dy
                if 0 <= n_x < height and 0 <= n_y < width and pixel_condition(grid_map[n_x, n_y]):
                    weight = np.sqrt(dx ** 2 + dy ** 2)  # Weight as intensity difference
                    G.add_edge((x, y), (n_x, n_y), weight=weight)

        return G

    def _mstc_star(self, graph, cap, obs_graph=nx.Graph(), is_write=False, is_show=False):
        R = self._robot_positions
        k = len(R)

        planner = MSTCStarPlanner(graph, k, R, cap, True)
        plans = planner.allocate()
        paths, weights = planner.simulate(plans)
        return paths, weights
    
    def get_path_in_grid(self):
        return self._result_paths
    
    def get_real_path(self):
        real_paths = []
        for path in self._result_paths:
            tmp_path = []
            for wp in path:
                p_g = np.array([[wp[0]], [wp[1]]])
                p_r = self._map_config["ratio"] * self._map_config["R^r_g"] @ p_g + self._map_config["p^r_g"]
                tmp_path.append((p_r[0].item(), p_r[1].item()))
            real_paths.append(tmp_path)
        return real_paths

if __name__ == "__main__":
    img = cv2.imread("gs.pgm", cv2.IMREAD_GRAYSCALE)
    robot_positions = [(50, 50), (50, 60), (50, 70)]

    map_config = {
        "ratio": 0.5,
        "R^r_g": np.eye(2),
        "p^r_g": np.zeros([2, 1]) 
    }

    path_cover_map = PathCoveragePlanner(img, robot_positions, map_config)

    paths = path_cover_map.get_real_path()
    


