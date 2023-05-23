# Import necessary modules
import pygame  # For GUI
import math  # For mathematical operations
from queue import PriorityQueue  # For efficient queuing operations

# Set up grid and window dimensions
WIDTH = 500  # Width of the window
WIN = pygame.display.set_mode((WIDTH, WIDTH))  # Initialize a window of width WIDTH
pygame.display.set_caption("A* Path Finding Algorithm")  # Set window title

# Define colors to be used for different types of nodes
RED = (255, 0, 0)  # Color for closed nodes
GREEN = (0, 255, 0)  # Color for open nodes
BLUE = (0, 0, 255)  # Unused color
YELLOW = (255, 255, 0)  # Unused color
WHITE = (255, 255, 255)  # Default color for nodes
BLACK = (0, 0, 0)  # Color for barrier nodes
PURPLE = (128, 0, 128)  # Color for path nodes
ORANGE = (255, 165 ,0)  # Color for start node
GREY = (128, 128, 128)  # Color for grid lines
TURQUOISE = (64, 224, 208)  # Color for end node

# Define Node class
class Node:
    # Initializer
    def __init__(self, row, col, width, total_rows):
        self.row = row  # Row of node in grid
        self.col = col  # Column of node in grid
        self.x = row * width  # X-coordinate for drawing the node
        self.y = col * width  # Y-coordinate for drawing the node
        self.color = WHITE  # Initial color is white
        self.neighbors = []  # List of neighboring nodes
        self.width = width  # Width of node for drawing
        self.total_rows = total_rows  # Total rows in the grid

    # Return node's position
    def get_pos(self):
        return self.row, self.col

    # Check functions for node's current status
    def is_closed(self):
        return self.color == RED

    def is_open(self):
        return self.color == GREEN

    def is_barrier(self):
        return self.color == BLACK

    def is_start(self):
        return self.color == ORANGE

    def is_end(self):
        return self.color == TURQUOISE

    # Reset the node color to default (white)
    def reset(self):
        self.color = WHITE

    # Make functions to change node's status
    def make_start(self):
        self.color = ORANGE

    def make_closed(self):
        self.color = RED

    def make_open(self):
        self.color = GREEN

    def make_barrier(self):
        self.color = BLACK

    def make_end(self):
        self.color = TURQUOISE

    def make_path(self):
        self.color = PURPLE

    # Draw the node
    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    # Update neighboring nodes
    def update_neighbors(self, grid):
        self.neighbors = []
        # Check the node below
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier(): # DOWN
            self.neighbors.append(grid[self.row + 1][self.col])

        # Check the node above
        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier(): # UP
            self.neighbors.append(grid[self.row - 1][self.col])

        # Check the node to the right
        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier(): # RIGHT
            self.neighbors.append(grid[self.row][self.col + 1])

        # Check the node to the left
        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier(): # LEFT
            self.neighbors.append(grid[self.row][self.col - 1])

# Heuristic function for A* (Manhattan distance)
def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)

# Function to recreate the path after A* finds the destination
def reconstruct_path(came_from, current, draw):
    while current in came_from:
        current = came_from[current]
        current.make_path()
        draw()

# The main A* algorithm
def algorithm(draw, grid, start, end):
    # Some setup
    count = 0
    open_set = PriorityQueue()  # Nodes to be evaluated
    open_set.put((0, count, start))  # Add start node
    came_from = {}  # Keep track of path
    # Set all g_score to infinity initially
    g_score = {node: float("inf") for row in grid for node in row}
    g_score[start] = 0  # g_score of start node is 0
    # Set all f_score to infinity initially
    f_score = {node: float("inf") for row in grid for node in row}
    # f_score of start node is the heuristic distance from start to end
    f_score[start] = h(start.get_pos(), end.get_pos())

    open_set_hash = {start}  # Efficient way to check if a node is in the open set

    # As long as there are nodes to be evaluated
    while not open_set.empty():
        # Quit pygame if user closes window
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        # Get the node with the lowest f_score value
        current = open_set.get()[2]
        open_set_hash.remove(current)

        # If the current node is the end node, we have found the path
        if current == end:
            reconstruct_path(came_from, end, draw)  # Recreate the path
            end.make_end()  # Ensure end node is visually distinct
            return True  # Return True as we have found the path

        # If the current node is not the end node, check its neighbors
        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1  # Calculate potential g_score

            # If this path to the neighbor node is better than any previous ones, update the path
            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current  # This node can be reached optimally from the current node
                g_score[neighbor] = temp_g_score  # Update g_score
                # Update f_score - main criteria for choosing which node to visit next
                f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
                if neighbor not in open_set_hash:  # If the neighbor node is not in the open set, add it
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))  # Add to priority queue
                    open_set_hash.add(neighbor)  # Add to hash set
                    neighbor.make_open()  # Visually depict as an open node

        draw()  # Draw the current state of the grid

        if current != start:  # If the current node is not the start node, close it as we are done with it
            current.make_closed()

    return False  # If no path is found, return False

# Function to create the grid
def make_grid(rows, width):
    grid = []  # The grid
    gap = width // rows  # Size of each node
    for i in range(rows):  # For each row
        grid.append([])  # Create a new row
        for j in range(rows):  # For each column
            node = Node(i, j, gap, rows)  # Create a new node
            grid[i].append(node)  # Append the node to the grid row

    return grid  # Return the grid

# Function to draw the grid lines
def draw_grid(win, rows, width):
    gap = width // rows  # Size of each node
    for i in range(rows):  # For each row
        pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))  # Draw horizontal grid line
        for j in range(rows):  # For each column
            pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))  # Draw vertical grid line

# Function to draw the current state of the entire grid
def draw(win, grid, rows, width):
    win.fill(WHITE)  # Fill the entire window with white

    for row in grid:  # For each row in grid
        for node in row:  # For each node in row
            node.draw(win)  # Draw the node

    draw_grid(win, rows, width)  # Draw the grid lines
    pygame.display.update()  # Update the display

# Function to get the position of the node that is clicked
def get_clicked_pos(pos, rows, width):
    gap = width // rows  # Size of each node
    y, x = pos

    row = y // gap  # Row of clicked node
    col = x // gap  # Column of clicked node

    return row, col  # Return the position of the clicked node

# Main function
def main(win, width):
    ROWS = 50  # Number of rows in the grid
    grid = make_grid(ROWS, width)  # Create the grid

    start = None  # Start node
    end = None  # End node

    run = True  # Whether the game is running
    started = False  # Whether the algorithm has started
    while run:  # While the game is running
        draw(win, grid, ROWS, width)  # Draw the current state of the grid
        for event in pygame.event.get():  # For each event
            if event.type == pygame.QUIT:  # If user closes window, quit the game
                run = False

            if pygame.mouse.get_pressed()[0]:  # If left mouse button is clicked
                pos = pygame.mouse.get_pos()  # Get position of click
                row, col = get_clicked_pos(pos, ROWS, width)  # Get position of clicked node
                node = grid[row][col]  # Get the clicked node
                if not start and node != end:  # If start node doesn't exist, make clicked node the start node
                    start = node
                    start.make_start()

                elif not end and node != start:  # If end node doesn't exist, make clicked node the end node
                    end = node
                    end.make_end()

                elif node != end and node != start:  # If node is neither start nor end, make it a barrier
                    node.make_barrier()

            elif pygame.mouse.get_pressed()[2]:  # If right mouse button is clicked, reset the clicked node
                pos = pygame.mouse.get_pos()  # Get position of click
                row, col = get_clicked_pos(pos, ROWS, width)  # Get position of clicked node
                node = grid[row][col]  # Get the clicked node
                node.reset()  # Reset the node
                if node == start:  # If it was the start node, remove start
                    start = None
                elif node == end:  # If it was the end node, remove end
                    end = None

            if event.type == pygame.KEYDOWN:  # If a key is pressed
                if event.key == pygame.K_SPACE and start and end:  # If SPACE is pressed and both start and end exist
                    for row in grid:
                        for node in row:
                            node.update_neighbors(grid)  # Update the neighbors of all nodes

                    algorithm(lambda: draw(win, grid, ROWS, width), grid, start, end)  # Run the algorithm

                if event.key == pygame.K_c:  # If C is pressed, clear the entire grid
                    start = None  # Remove start
                    end = None  # Remove end
                    grid = make_grid(ROWS, width)  # Reset the grid

    pygame.quit()

# Run the main function
if __name__ == "__main__":
    main(WIN, WIDTH)
