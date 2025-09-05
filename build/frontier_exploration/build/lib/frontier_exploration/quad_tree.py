# quad_tree.py
class QuadTreeNode:
    def __init__(self, x, y, size, occupancy):
        self.x = x
        self.y = y
        self.size = size
        self.occupancy = occupancy
        self.children = []

    def is_leaf(self):
        return len(self.children) == 0


def build_quadtree(grid, x, y, size):
    """Recursively build a quadtree from an occupancy grid."""
    if size == 1:
        return QuadTreeNode(x, y, size, grid[y][x])

    half = size // 2
    children = [
        build_quadtree(grid, x, y, half),
        build_quadtree(grid, x + half, y, half),
        build_quadtree(grid, x, y + half, half),
        build_quadtree(grid, x + half, y + half, half)
    ]

    occupancies = [child.occupancy for child in children]
    if all(o == occupancies[0] for o in occupancies):
        return QuadTreeNode(x, y, size, occupancies[0])
    else:
        node = QuadTreeNode(x, y, size, None)
        node.children = children
        return node


def find_frontiers_quadtree(node, grid):
    """Traverse the quadtree to find frontiers."""
    frontiers = []
    stack = [node]

    while stack:
        current = stack.pop()
        if current.is_leaf() and current.occupancy == 0:
            neighbors = [
                (current.x - 1, current.y),
                (current.x + 1, current.y),
                (current.x, current.y - 1),
                (current.x, current.y + 1)
            ]
            if any(
                0 <= nx < len(grid[0]) and 0 <= ny < len(grid) and grid[ny][nx] == -1
                for nx, ny in neighbors
            ):
                frontiers.append((current.x, current.y))
        else:
            stack.extend(current.children)

    return frontiers

import matplotlib.pyplot as plt
import matplotlib.patches as patches

def save_quadtree_image(node, filename):
    """Visualizes the QuadTree structure and saves it as an image."""
    fig, ax = plt.subplots(figsize=(8, 8))
    
    def draw_node(node, ax):
        """Recursive function to draw QuadTree nodes."""
        if node.is_leaf():
            # Color based on occupancy
            color = (
                'white' if node.occupancy == 0 else
                'black' if node.occupancy == 1 else
                'gray'
            )
            rect = patches.Rectangle(
                (node.x, node.y),
                node.size, node.size,
                linewidth=0.5,
                edgecolor='black',
                facecolor=color,
                alpha=0.6
            )
            ax.add_patch(rect)
        else:
            for child in node.children:
                draw_node(child, ax)
    
    # Draw QuadTree
    draw_node(node, ax)
    
    # Configure plot
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(0, node.size)
    ax.set_ylim(0, node.size)
    ax.set_title('QuadTree Occupancy Map')
    ax.set_xlabel('X (cells)')
    ax.set_ylabel('Y (cells)')
    plt.gca().invert_yaxis()  # Invert Y-axis for proper orientation
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.savefig(filename, dpi=300)
    plt.close()
    print(f"QuadTree image saved as {filename}")


def main(args=None):
    print("Quad Tree Node module loaded")

if __name__ == '__main__':
    main()