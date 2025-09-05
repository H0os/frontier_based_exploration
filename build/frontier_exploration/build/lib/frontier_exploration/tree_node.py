#Algorithm Pass it a tree node. if it is a empty tree we create a node at the position of the robot at the center of the map, once we have a root node we will recursively make children nodes at a fix length up, down, left and right ensuring a node is not there already there, we check if these nodes are in known spaces if they are we continue to make children nodes from them, if they are not we stop there. 
from collections import deque
import multiprocessing
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_matrix, quaternion_from_euler
import cv2


class TreeNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.searched = False
        self.children = []

    def add_child(self, grid, child_node):
        self.children.append(child_node)

    def is_leaf(self):
        return len(self.children) == 0
    
grid_size= int(20)

def get_grid_size():
    return grid_size

def is_in_free_space(grid, x, y):
    """
    Check that (x, y) is within grid bounds AND is free space (0).
    Adjust indexing if your grid is row-major with grid[y, x].
    """
    if 0 <= y < grid.shape[0] and 0 <= x < grid.shape[1]:
        return grid[y, x] == 0
    return False

        
def build_tree(grid, root, node_set, offset_x, offset_y):
    if offset_x != 0 or offset_y != 0:
        if not root:
            return
        
        queue = deque([root])

        while queue:
            
            node = queue.popleft()
            #print(f"currently at node: {node.x}, {node.y}")
            try:
                node_set.remove((node.x, node.y))
            except:
                print("no node to remove!")
            node.x = node.x - offset_x
            node.y = node.y - offset_y
            node_set.add((node.x, node.y))
            for child in node.children:
                queue.append(child)
        


    if not root:
        return
    
    queue = deque([root])
    unserached_nodes = []

    while queue:
        
        node = queue.popleft()
        #print(f"currently at node: {node.x}, {node.y}")
        node_set.add((node.x, node.y))
        if not node.searched:
            unserached_nodes.append(node)

        if is_in_free_space(grid, node.x, node.y):
            create_node(grid, node, node_set, 0, 1)
            create_node(grid, node, node_set, 1, 0)
            create_node(grid, node, node_set, 0, -1)
            create_node(grid, node, node_set, -1, 0)
            create_node(grid, node, node_set, 1, 1)
            create_node(grid, node, node_set, 1, -1)
            create_node(grid, node, node_set, -1, -1)
            create_node(grid, node, node_set, -1, 1)

        for child in node.children:
            node_set.add((child.x, child.y))
            queue.append(child)
    
    return node_set, unserached_nodes

def create_node(grid,  node, node_set, offset_x, offset_y):
        new_x = node.x + (grid_size * offset_x)
        new_y = node.y + (grid_size * offset_y)
        if (new_x, new_y) not in node_set:
                node.add_child(grid, TreeNode(new_x, new_y))
                node.searched = False
                #print(f"node created at: {new_x}, {new_y}")

def search(grid, unsearched_nodes, resolution , origin):
    
    num_workers = 12
    with multiprocessing.Pool(num_workers) as pool:
        results = pool.map(parallel_search_tree, ((grid, n, resolution, origin) for n in unsearched_nodes))
    
    all_frontiers = []
    empty_arrays = []
    for (found_frontiers, data) in results:
        if found_frontiers:
            all_frontiers.extend(data) 
        else:
            empty_arrays.append(data)
   # print(all_frontiers)
    #print(empty_arrays)
    for node in unsearched_nodes:
        if (node.x, node.y) in empty_arrays:
            node.searched = True

    # if not root:
    #     return
    # queue= deque([root])
    
    # while queue:

    #     node = queue.popleft()
    #     if (node.x, node.y) in empty_arrays: #and not node.is_leaf():
    #         node.searched = True 
    #         #print(f"node ({node.x}, {node.y}) will not be searched again")
              

    #     for child in node.children:
    #         queue.append(child)

    #print(frontiers_list)
    print("Search complete for all nodes")
    return all_frontiers, len(unsearched_nodes)


def parallel_search_tree(args):
    grid, node, resolution, origin = args

    frontiers = search_tree(grid, node, resolution, origin)

    if frontiers:
        return (True, frontiers)
    else:
        return(False, (node.x, node.y))

def search_tree(grid, node, resolution, origin):
    #print( f"subprocess: ({node.x}, {node.y})")
    height, width = grid.shape
    frontiers = set()

    x_min, x_max = max(0, node.x - (grid_size // 2)), min(width, node.x + (grid_size // 2))
    y_min, y_max = max(0, node.y - (grid_size // 2)), min(height, node.y + (grid_size // 2))
    #print(f"searching area {x_min},{y_min} to {x_max},{y_max}")
    for y in range(y_min, y_max):
        #print( f"subprocess: ({node.x}, {node.y}) working!")
        for x in range(x_min, x_max):
            if grid[y, x] == 0:  # Free space
                # Check if any adjacent cell is unknown (-1), making it a frontier
                neighbors = [
                    (y-1, x), (y+1, x), (y, x-1), (y, x+1)  # Up, Down, Left, Right
                ]
                for ny, nx in neighbors:
                    if 0 <= ny < height and 0 <= nx < width and grid[ny, nx] == -1:
                        world_x, world_y = grid_to_world(x, y, resolution, origin)
                        frontiers.add((world_x, world_y))
                        break  # Stop checking neighbors if already identified
     
    return frontiers 

def remove_searched_nodes_bfs(root):
    """
    Traverse the tree in BFS order, removing any child node that has
    node.searched == True. The removed node's children are then reattached
    to the parent.

    :param root: The root TreeNode of the tree.
    """
    if not root:
        return

    queue = deque([root])

    while queue:
        node = queue.popleft()

        # Collect a new list of children for the current node
        updated_children = []
        for child in node.children:
            if child.searched and node.searched:
                # Instead of keeping this child, reattach its children
                # to the current node
                updated_children.extend(child.children)
            else:
                updated_children.append(child)

        # Replace the old children list with the updated one
        node.children = updated_children

        # Enqueue the newly attached children for BFS
        for child in node.children:
            queue.append(child)


def grid_to_world(x, y, resolution, origin):
    """Convert grid coordinates to world coordinates."""
    if origin is None or resolution is None:
        print("Map origin or resolution is not set.")
        return None, None

    # Convert grid to relative coordinates
    

    # Apply map origin transformation
    origin = origin.position
    
    world_x = origin.x + x * resolution
    world_y = origin.y + y * resolution
    return world_x, world_y

    


import matplotlib.pyplot as plt
import numpy as np
from collections import deque

def visualize_tree_on_map(grid, root, out_filename="tree_overlay.png"):
    """
    Overlays the tree on top of a visualization of the occupancy grid.
    - Grid cells with value 0 (free) are white.
    - Cells with value 100 or >0 (occupied) are black.
    - Cells with value -1 (unknown) are gray.
    - Tree nodes are drawn as circles:
        Green if node.searched == True
        Red if node.searched == False
    - Edges (parent->child) are drawn in the same color as the parent node.
    
    :param grid: 2D numpy array representing the occupancy grid
    :param root: The root TreeNode of the tree to visualize
    :param out_filename: Name of the file to save the resulting image
    """
    height, width = grid.shape

    # 1. Create an RGB image from the occupancy grid
    #    White for free (0), black for occupied, gray for unknown (-1).
    image = np.zeros((height, width, 3), dtype=np.uint8)
    for y in range(height):
        for x in range(width):
            if grid[y, x] == 0:
                image[y, x] = (255, 255, 255)   # free space -> white
            elif grid[y, x] == -1:
                image[y, x] = (128, 128, 128)   # unknown -> gray
            else:
                image[y, x] = (0, 0, 0)         # occupied -> black

    # 2. Traverse the tree (BFS or DFS) to plot nodes and edges
    queue = deque([root])

    while queue:
        node = queue.popleft()

        # Determine node color:
        #   green if node.searched == True, else red
        color = (0, 255, 0) if node.searched else (0, 0, 255)

        # Draw the node as a small circle
        # Make sure (node.x, node.y) are within image bounds before drawing
        if 0 <= node.x < width and 0 <= node.y < height:
            cv2.circle(image, (node.x, node.y), 2, color, thickness=-1)

        # For each child, draw a line from parent to child (in parent's color),
        # and enqueue the child for processing
        for child in node.children:
            queue.append(child)
            # Also check bounds for child before drawing the line
            if (0 <= child.x < width) and (0 <= child.y < height) and (0 <= node.x < width) and (0 <= node.y < height):
                cv2.line(image, (node.x, node.y), (child.x, child.y), color, thickness=1)

    # 3. Save the resulting image
    cv2.imwrite(out_filename, image)
    print(f"Tree visualization saved to {out_filename}")


def main(args=None):
    print("Tree Node module loaded")

if __name__ == '__main__':
    main()