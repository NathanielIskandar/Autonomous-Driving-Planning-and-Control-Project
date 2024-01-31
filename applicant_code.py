import numpy as np
import pygame

class Node:
    def __init__(self, point):
        self.point = np.array(point)
        self.parent = None
        self.cost = 0.0
        self.children = []
        
class RRTStar(object):
    def __init__(self, screen, start, goal, blue_cones, yellow_cones, POINT_RADIUS, neighbor_radius, width, height, step_size=10, max_iter=50):
        self.screen = screen
        self.start = Node(start)
        self.goal = Node(goal)
        self.blue_cones = blue_cones
        self.yellow_cones = yellow_cones
        self.POINT_RADIUS = POINT_RADIUS
        self.neighbor_radius = neighbor_radius
        self.width = width
        self.height = height
        self.step_size = step_size 
        self.max_iter = max_iter
        self.nodes = [self.start]
        self.COLOR = (30, 225, 112)
        
    #HELPFUL GENERAL FUNCTIONS 
    #=============================================================================
    def distance(self, point1, point2):
        return np.linalg.norm(np.array(point1) - np.array(point2))
    
    #COMMENTED OUT FOR NOW BECAUSE LINE CROSSING LOGIC IS FAULTY [from outside resource reference]
    #==============================================================================================
    def ccw(self, A, B, C):
        """Checks if the points A, B, and C are listed in a counterclockwise order."""
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    def intersect(self, p1, q1, p2, q2):
        """Returns True if line segments 'p1q1' and 'p2q2' intersect."""
        return (self.ccw(p1, p2, q2) != self.ccw(q1, p2, q2)) and (self.ccw(p1, q1, p2) != self.ccw(p1, q1, q2))

    def line_intersect(self, p1, p2, p3, p4):
        """Check if the line segment from p1 to p2 intersects with the line segment from p3 to p4."""
        return self.intersect(p1, p2, p3, p4)
    #=============================================================================


    #BEGIN RRT*-SPECIFIC FUNCTIONS
    #=============================================================================
    def add_connect_new_node(self, nearest_node, point):
        #ADJACENCY LIST!
        #Defining a new node
        new_node = Node(point)
        new_node.parent = nearest_node
        new_node.cost = nearest_node.cost + self.distance(point, nearest_node.point)
        nearest_node.children.append(new_node)

        self.nodes.append(new_node) #apprend to the list of nodes
        return new_node

    def sample_point(self):
        x = np.random.randint(self.POINT_RADIUS, self.width - self.POINT_RADIUS)
        y = np.random.randint(self.POINT_RADIUS, self.height - self.POINT_RADIUS)
        return (x, y)
    
    def find_nearest_node(self, point):
        closest_node = None
        min_dist = float('inf')
        for node in self.nodes:
            dist = self.distance(node.point, point)
            if dist < min_dist:
                min_dist = dist
                closest_node = node
        return closest_node

    def steer(self, from_node, to_point, step_size):
        direction = np.array(to_point) - from_node.point
        distance = np.linalg.norm(direction)
        direction = direction / distance  
        new_point = from_node.point + direction * step_size
        return new_point

    # def steer(self, from_node, to_point, EPSILON):
    #     if self.distance(from_node.point, to_point) < EPSILON:
    #         # If the distance to the random node is less than EPSILON, just return the random node.
    #         return to_point
    #     else:
    #         # Otherwise, take a step from the nearest node towards the random node.
    #         theta = atan2(to_point[1] - from_node.point[1], to_point[0] - from_node.point[0])
    #         new_point = (from_node.point[0] + EPSILON * cos(theta), from_node.point[1] + EPSILON * sin(theta))
    #         return new_point

    

    #COMMENTED OUT FOR NOW BECAUSE CROSSES_BOUNDARY IS FAULTY
    def crosses_boundary(self, from_node, to_point):
        for i in range(len(self.blue_cones) - 1):
            if self.line_intersect(from_node.point, to_point, self.blue_cones[i], self.blue_cones[i + 1]):
                return True
        for i in range(len(self.yellow_cones) - 1):
            if self.line_intersect(from_node.point, to_point, self.yellow_cones[i], self.yellow_cones[i + 1]):
                return True
        return False


    def find_neighbors(self, new_node, neighbor_radius):
        # If a node is within a radius range, add it to neighbors list
        arr = []
        for node in self.nodes:
            if node != new_node and self.distance(node.point, new_node.point) < neighbor_radius:
                arr.append(node)
        return arr
        
    
    def select_parent(self, curr_node, new_node):
        # If the cost of the new node + distance from the curr node to that node is less than curr node's cost, replace
        #instead of distnace, it's the cost
        if  new_node.cost + self.distance(curr_node.point, new_node.point) < curr_node.cost:
            curr_node.cost = new_node.cost + self.distance(curr_node.point, new_node.point) #let node.cost be the cheaper one
            curr_node.parent = new_node
        return
    
    def draw_connection_to_parent(self, parent):
        #draw the parent point
        pygame.draw.circle(self.screen, self.COLOR, parent.point, self.POINT_RADIUS)

        for child in parent.children:
            #draw the child point
            pygame.draw.circle(self.screen, self.COLOR, child.point, self.POINT_RADIUS)
            #draw the line between them
            pygame.draw.line(self.screen, self.COLOR, parent.point, child.point, 2)
            pygame.display.flip()

            self.draw_connection_to_parent(child)
        

        



            

    #=============================================================================
                
    

    #GENERATE PATH
    def generate_path(self):
        for i in range(self.max_iter):
            print("iteration #:", i)
            sampled_point = self.sample_point() #Sampe a random point
            nearest_node = self.find_nearest_node(sampled_point) #FInd the nearest node to this random point
            projected_point = self.steer(nearest_node, sampled_point, self.step_size) #Projecting a node towards the direction of the random point
            if projected_point is None:
                continue
            # #Check if our projected node is valid  // COMMENTED OUT FOR NOW BECAUSE CROSSES BOUNDARY IS FAULTY
            # if self.crosses_boundary(nearest_node, projected_point): # If boundary is crossed, try again
            #     continue #skip everything and regenerate a point

            #If the projected point is valid, then create new node
            new_node = self.add_connect_new_node(nearest_node, projected_point)
            self.draw_connection_to_parent(self.start)

            #if new_node is not None:  # Only draw if the new node was added
                #pygame.draw.circle(self.screen, self.COLOR, tuple(new_node.point.astype(int)), self.POINT_RADIUS)
                #pygame.display.flip() 

            #Finding neighbors to nearest node within a radius
            neighbors = self.find_neighbors(new_node, self.neighbor_radius) #neighbors is a list

            for neighbor in neighbors:
                self.select_parent(neighbor, new_node)

            # Extract the path from the nodes
            print("Length of Nodes.list = ", len(self.nodes))
            print("         ")
        path = []
        for node in self.nodes:
            point = node.point
            print(point)
            path.append(point)
        return path[::-1]

            

                
#def planner(blue_cones, yellow_cones, screen, SCREEN_WIDTH, SCREEN_HEIGHT):
def planner(blue_cones, yellow_cones, screen, POINT_RADIUS, SCREEN_WIDTH, SCREEN_HEIGHT):
    """plans a path through the track given the blue and yellow cones.

	Args:
		blue_cones (list): blue (left) cone locations. shape (n, 2).
		yelllow_cones (list): yellow (right) cone locations. shape (n, 2).

	Returns:
		list: output path, with shape (n, 2)
	"""
    ###### YOUR CODE HERE ######
	# fill out this function to make it work
	# as described in the docstring above

    start = np.array([int((blue_cones[0][0] + yellow_cones[0][0]) / 2), int((blue_cones[0][1] + yellow_cones[0][1]) / 2)])
    goal = np.array([int((blue_cones[-2][0] + yellow_cones[-2][0]) / 2), int((blue_cones[-2][1] + yellow_cones[-2][1]) / 2)])

    neighbor_radius = 500

    #without Pygame
    rrt_star = RRTStar(screen, start, goal, blue_cones, yellow_cones, POINT_RADIUS, neighbor_radius, SCREEN_WIDTH, SCREEN_HEIGHT, step_size=30, max_iter=1000)
    
    
    ret = rrt_star.generate_path()
    print("         ")
    print("Generated RRT points: ")
    print(ret)

    return ret