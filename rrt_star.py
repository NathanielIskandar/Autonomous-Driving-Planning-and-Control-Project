import numpy as np
from shapely.geometry import LineString
import pygame

class Node:
    # A class representing a node in the RRT and RRT* algorithm
    def __init__(self, point):
        self.point = np.array(point)
        self.parent = None
        self.cost = 0.0
        self.children = []

class Obstacle(object):
    # A class representing an obstacle defined by a list of coordinates
    def __init__(self, coordinates_list):
        self.boundary = LineString(coordinates_list) # The shapely LineString representing the obstacle
    
    def check_intersect(self, from_point, to_point):
        # Check if a line from from_point to to_point intersects with the obstacle
        line = LineString([from_point, to_point])
        return self.boundary.intersects(line)

        
class RRTStar(object):
    # A class implementing the RRT* algorithm
    def __init__(self, racetrack, screen, start, goal, blue_cones, yellow_cones, POINT_RADIUS, SCREEN_WIDTH, SCREEN_HEIGHT, neighbor_radius, radius_from_goal = 20,step_size=10):    
        # Canvas setup
        self.racetrack = racetrack # The RaceTrack object containing the track information
        self.screen = screen # The pygame screen object for drawing
        self.blue_cones = blue_cones # Lists of coordinates for the blue track boundaries
        self.blue_cones.append(blue_cones[0]) # Ensure loop closure
        self.yellow_cones = yellow_cones # Lists of coordinates for the yellow track boundaries
        self.yellow_cones.append(yellow_cones[0]) # Ensure loop closure
        self.SCREEN_WIDTH = SCREEN_WIDTH 
        self.SCREEN_HEIGHT = SCREEN_HEIGHT 
        self.COLOR = (100, 100, 100)
        self.BACKGROUND_COLOR = (255, 255, 255)

        # Nodes setup
        self.start_node = Node(start)
        self.goal_node = Node(goal)
        self.POINT_RADIUS = POINT_RADIUS # The radius of the points to be drawn
        self.neighbor_radius = neighbor_radius # The maximum radius to look for neighbor nodes
        self.step_size = step_size # The maximum distance between nodes
        self.radius_from_goal = radius_from_goal # The radius within which the goal is considered reached
        self.nodes = [self.start_node]
        
        # Obstacle setup
        self.blue_cones_obstacle = Obstacle(blue_cones) #blue cones obstacle instantiantion
        self.yellow_cones_obstacle = Obstacle(yellow_cones) #yellow cones obstacle instantiation
        self.absolute_line_left_point = np.array( [blue_cones[-2][0], blue_cones[-2][1]] ) #blue_cones[-2] represent the point where the goal node is
        self.absolute_line_right_point = np.array( [yellow_cones[-2][0], yellow_cones[-2][1]] )
        self.start_absolute_line_bound = [self.absolute_line_left_point, self.absolute_line_right_point]
        self.starting_line = Obstacle(self.start_absolute_line_bound)

    # Utility function to calculate Euclidean distance between two points
    def distance(self, point1, point2):
        return np.linalg.norm(np.array(point1) - np.array(point2))

    # Adds a new node to the tree, connecting it to the nearest node
    def add_connect_new_node(self, nearest_node, point):
        new_node = Node(point)
        new_node.parent = nearest_node
        new_node.cost = nearest_node.cost + self.distance(point, nearest_node.point)
        nearest_node.children.append(new_node)
        self.nodes.append(new_node)
        return new_node

    # Generates a random point within the defined boundaries of the screen.
    def sample_point(self):
        x = np.random.randint(self.POINT_RADIUS, self.SCREEN_WIDTH - self.POINT_RADIUS)
        y = np.random.randint(self.POINT_RADIUS, self.SCREEN_HEIGHT - self.POINT_RADIUS)
        return (x, y)
    
    # Finds the node in the RRT Star tree that is closest to the given point.
    def find_nearest_node(self, point):
        closest_node = None
        min_dist = float('inf')
        for node in self.nodes:
            dist = self.distance(node.point, point)
            if dist < min_dist:
                min_dist = dist
                closest_node = node
        return closest_node

    # Generates a new point along the direction from a node towards a target point, constrained by a step size.
    def steer(self, from_node, to_point, step_size):
        direction = np.array(to_point) - from_node.point
        distance = np.linalg.norm(direction)
        direction = direction / distance  
        new_point = from_node.point + direction * step_size
        return new_point

    # Checks whether a direct path from one node to another crosses any of the defined boundaries.
    def crosses_boundary(self, from_node, to_point):
        if not self.blue_cones_obstacle.check_intersect(from_node.point, to_point) and \
           not self.yellow_cones_obstacle.check_intersect(from_node.point, to_point) and \
           not self.starting_line.check_intersect(from_node.point, to_point):
            return False
        return True

    # Identifies all nodes within a certain radius of a new node that could potentially be its parent.
    def find_neighbors(self, new_node, neighbor_radius):
        arr = []
        for node in self.nodes:
            if node != new_node and self.distance(node.point, new_node.point) < neighbor_radius:
                arr.append(node)
        return arr
        
    # Chooses the lowest-cost node from the neighboring nodes to be the parent of the current node.
    def select_parent(self, curr_node, new_node):
        if  new_node.cost + self.distance(curr_node.point, new_node.point) < curr_node.cost and not self.crosses_boundary(curr_node, new_node.point):
            print("Found a better parent")
            curr_node.cost = new_node.cost + self.distance(curr_node.point, new_node.point) #let node.cost be the cheaper one
            current_parent = curr_node.parent
            curr_node.parent = new_node
            new_node.children.append(curr_node)
            current_parent.children.remove(curr_node)
        return
    
    # Draws a visual connection between a parent node and all its children on the pygame screen.
    def draw_connection_to_parent(self, parent):
        pygame.draw.circle(self.screen, self.COLOR, parent.point, self.POINT_RADIUS)
        for child in parent.children:
            pygame.draw.circle(self.screen, self.COLOR, child.point, self.POINT_RADIUS)
            pygame.draw.line(self.screen, self.COLOR, parent.point, child.point, 2)
            self.draw_connection_to_parent(child)

    # Checks if any node within the goal radius has been reached. If so, the goal is considered achieved.
    def found_node_within_goal_radius(self):
        nearest_node_to_goal = self.find_nearest_node(self.goal_node.point)
        pygame.draw.circle(self.screen, (0,0,200), tuple(nearest_node_to_goal.point.astype(int)), self.POINT_RADIUS)
        if self.distance(nearest_node_to_goal.point, self.goal_node.point) <= self.radius_from_goal:
            return True
        return False  
    
    # Create visualizations on the screen
    def visualize_elements(self):
        # Draw the absolute line points and the connecting line.
        pygame.draw.circle(self.screen, (255, 0, 255), self.absolute_line_left_point, self.POINT_RADIUS)
        pygame.draw.circle(self.screen, (255, 0, 255), self.absolute_line_right_point, self.POINT_RADIUS)
        pygame.draw.line(self.screen, (255, 0, 255), self.absolute_line_left_point, self.absolute_line_right_point, 2)

        # Draw the goal point, track boundaries, starting point, and direction.
        pygame.draw.circle(self.screen, (0,0,200), tuple(self.goal_node.point.astype(int)), self.POINT_RADIUS)
        self.racetrack.draw_yellow_cones()
        self.racetrack.draw_blue_cones()
        self.racetrack.draw_start()
        self.racetrack.draw_start_directon()
        
        # Visualize the connections in the tree.
        self.draw_connection_to_parent(self.start_node)
        pygame.display.flip()

    # RRT* Optimization of selecting a more cost-effective path
    def optimize_path(self, new_node):
        # Find neighbors within a defined radius of the new node.
        neighbors = self.find_neighbors(new_node, self.neighbor_radius)
        
        # For each neighbor, attempt to find a more optimal path through the new node.
        for neighbor in neighbors:
            self.select_parent(neighbor, new_node)

    # Extracts the best path attainable through backtracking from the goal point
    def backtrack_path_from_goal(self):
        # Initialize path with the goal node point.
        path = []
        current = self.find_nearest_node(self.goal_node.point)
        
        # Backtrack from goal node to start node, adding each node's point to the path.
        while current.parent is not self.start_node:
            path.append(current.point)
            current = current.parent
        path.append(self.start_node.point)

        # Reverse the path to start from the start node and end at the goal node.
        return path[::-1]
    

    #GENERATE PATH
    def generate_path(self):
        while not self.found_node_within_goal_radius():
            self.screen.fill(self.BACKGROUND_COLOR) # Clear the screen for new drawing.
            
            # Sample a random point on the screen and find the nearest existing node to it.
            sampled_point = self.sample_point() 
            nearest_node = self.find_nearest_node(sampled_point) 
            
            # Steer from nearest node towards the sampled point up to a maximum step size.
            projected_point = self.steer(nearest_node, sampled_point, self.step_size) #Projecting a node towards the direction of the random point
            if projected_point is None or self.crosses_boundary(nearest_node, projected_point):
                continue # Skip if projected point is invalid or crosses a boundary.

            # Add a new valid node to the tree.
            new_node = self.add_connect_new_node(nearest_node, projected_point)

            # Visualize the boundary, goal point, track, and connections.
            self.visualize_elements()

            # Attempt to optimize the path by reselecting parents for nodes near the new node.
            self.optimize_path(new_node)

        # Return the best path generated by backtracking from the goal to the start
        return self.backtrack_path_from_goal()
    
    


        
def rrt_star_planner(racetrack, blue_cones, yellow_cones, screen, POINT_RADIUS, SCREEN_WIDTH, SCREEN_HEIGHT):
    """plans a path through the track given the blue and yellow cones.

	Args:
		blue_cones (list): blue (left) cone locations. shape (n, 2).
		yelllow_cones (list): yellow (right) cone locations. shape (n, 2).

	Returns:
		list: output path, with shape (n, 2)
	"""

    # Initialize the start and goal point from the given blue and yellow cones coordinate
    start = np.array([int((blue_cones[0][0] + yellow_cones[0][0]) / 2), int((blue_cones[0][1] + yellow_cones[0][1]) / 2)])
    goal = np.array([int((blue_cones[-2][0] + yellow_cones[-2][0]) / 2), int((blue_cones[-2][1] + yellow_cones[-2][1]) / 2)])

    # Generate an instance of RRTStar
    rrt_star = RRTStar(racetrack, screen, start, goal, blue_cones, yellow_cones, POINT_RADIUS, SCREEN_WIDTH, SCREEN_HEIGHT, neighbor_radius = 1000, step_size=30)
    
    # Generate the path using the RRT algorithm
    generated_path = rrt_star.generate_path()

    # Print the generated path for debugging
    print("Generated RRT STAR points: ")
    print(generated_path)

    return generated_path