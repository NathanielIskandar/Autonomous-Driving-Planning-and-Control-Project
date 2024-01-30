import numpy as np

class Node:
    def __init__(self, point):
        self.point = np.array(point)
        self.parent = None
        self.cost = 0.0

class RRTStar(object):
    def __init__(self, start, goal, blue_cones, yellow_cones, POINT_RADIUS, neighbor_radius, width, height, step_size=1.0, max_iter=50):
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
        
    #HELPFUL GENERAL FUNCTIONS 
    #=============================================================================
    def distance(self, point1, point2):
        return np.linalg.norm(np.array(point1) - np.array(point2))
    
    #COMMENTED OUT FOR NOW BECAUSE LINE CROSSING LOGIC IS FAULTY [from outside resource reference]
    #==============================================================================================
    # def ccw(self, A, B, C):
    #     """Checks if the points A, B, and C are listed in a counterclockwise order."""
    #     return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    # def intersect(self, p1, q1, p2, q2):
    #     """Returns True if line segments 'p1q1' and 'p2q2' intersect."""
    #     return (self.ccw(p1, p2, q2) != self.ccw(q1, p2, q2)) and (self.ccw(p1, q1, p2) != self.ccw(p1, q1, q2))

    # def line_intersect(self, p1, p2, p3, p4):
    #     """Check if the line segment from p1 to p2 intersects with the line segment from p3 to p4."""
    #     return self.intersect(p1, p2, p3, p4)
    #=============================================================================


    #BEGIN RRT*-SPECIFIC FUNCTIONS
    #=============================================================================
    def add_connect_new_node(self, nearest_node, point):
        #Defining a new node
        new_node = Node(point)
        new_node.parent = nearest_node
        new_node.cost = nearest_node.cost + self.distance(point, nearest_node.point)

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
            dist = np.linalg.norm(node.point - point)
            if dist < min_dist:
                min_dist = dist
                closest_node = node
        return closest_node

    def steer(self, from_node, to_point, step_size):
        direction = np.array(to_point) - from_node.point
        distance = np.linalg.norm(direction)
        if distance > step_size:
            direction = direction / distance  
            new_point = from_node.point + direction * step_size
        else:
            new_point = to_point
        return new_point
    

    #COMMENTED OUT FOR NOW BECAUSE CROSSES_BOUNDARY IS FAULTY
    # def crosses_boundary(self, from_node, to_point):
    #     for i in range(len(self.blue_cones) - 1):
    #         if self.line_intersect(from_node.point, to_point, self.blue_cones[i], self.blue_cones[i + 1]):
    #             return True
    #     for i in range(len(self.yellow_cones) - 1):
    #         if self.line_intersect(from_node.point, to_point, self.yellow_cones[i], self.yellow_cones[i + 1]):
    #             return True
    #     return False


    

    def find_neighbors(self, arr, new_node, neighbor_radius):
        # If a node is within a radius range, add it to neighbors list
        for node in self.nodes:
            if self.distance(node.point, new_node.point) < neighbor_radius:
                arr.append(node)
        return arr
        
    
    def select_parent(self, curr_node, new_node):
        # If the cost of the new node + distance from the curr node to that node is less than curr node's cost, replace
        if  new_node.cost + self.distance(curr_node.point, new_node.point) < curr_node.cost:
            curr_node.cost = new_node.cost + self.distance(curr_node.point, new_node.point) #let node.cost be the cheaper one
            new_node.parent = curr_node
        return
    #=============================================================================
                
    

    #GENERATE PATH
    def generate_path(self):
        for i in range(self.max_iter):
            sampled_point = self.sample_point() #Sampe a random point
            nearest_node = self.find_nearest_node(sampled_point) #FInd the nearest node to this random point
            projected_point = self.steer(nearest_node, sampled_point, self.step_size) #Projecting a node towards the direction of the random point
            
            # #Check if our projected node is valid  // COMMENTED OUT FOR NOW BECAUSE CROSSES BOUNDARY IS FAULTY
            # if self.crosses_boundary(nearest_node, projected_point): # If boundary is crossed, try again
            #     continue #skip everything and regenerate a point

            #If the projected point is valid, then create new node
            new_node = self.add_connect_new_node(nearest_node, projected_point)

            #Finding neighbors to nearest node within a radius
            neighbors = []
            neighbors = self.find_neighbors(neighbors, new_node, self.neighbor_radius)

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

            

                

def planner(blue_cones, yellow_cones):
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
    
    #NOTE: I THINK THE START SHOULD REFERENCE THE POINT AT WHICH DRAW_START AND DRAW_START_DIRECTION IS DRAWN
    #shouldve been where it starts. refer to the start_draw method below:
    # def draw_start(self):
    #     if len(self.path_points) > 0:
    #         pygame.draw.circle(self.screen, self.STARTCOLOR, self.path_points[0], self.POINT_RADIUS)

    # def draw_start_directon(self):
    #     if len(self.path_points) > 0:
    #         pygame.draw.line(self.screen, self.RED, self.path_points[0], self.path_points[0] + self.start_directon, 2)


    # Calculate the midpoint for the start between the first blue and yellow cones
    start = np.array((blue_cones[0][0] + yellow_cones[0][0]) // 2, (blue_cones[0][1] + yellow_cones[0][1]) // 2)

    # Calculate the midpoint for the goal between the second to last blue and yellow cones
    goal = np.array((blue_cones[-2][0] + yellow_cones[-2][0]) // 2, (blue_cones[-2][1] + yellow_cones[-2][1]) // 2)

    POINT_RADIUS = 5
    neighbor_radius = 500
    SCREEN_WIDTH = 800
    SCREEN_HEIGHT = 800
    rrt_star = RRTStar(start, goal, blue_cones, yellow_cones, POINT_RADIUS, neighbor_radius, SCREEN_WIDTH, SCREEN_HEIGHT, step_size=1.0, max_iter=1000)
    
    
    ret = rrt_star.generate_path()
    print("         ")
    print("Generated RRT points: ")
    print(ret)


    return [] # placeholder







   


    # def steer(self, from_node, to_point, step_size):
    #     direction = np.array(to_point) - from_node.point
    #     distance = np.linalg.norm(direction)
    #     if distance > step_size:
    #         direction = direction / distance  
    #         new_point = from_node.point + direction * step_size
    #     else:
    #         new_point = to_point

    #     #Defining a new node
    #     new_node = Node(new_point)
    #     new_node.parent = from_node
    #     new_node.cost = from_node.cost + np.linalg.norm(new_point - from_node.point)


    #     self.nodes.append(new_node) #apprend to the list of nodes
    #     return new_node


    #NOT NECESSARY BECAUSE WE HAVE NODE.COST ATTRIBUTE
    # def cost(self, node):
    #     # Calculates the cost of the current node to traverse up the parent
    #     # This is part of a bigger function that will check the cost of all nodes within a radius
    #     cost = 0
    #     root = node.start
    #     def traverse_up_parent(node):
    #         if node == root:
    #             return 0
    #         node = node.parent
    #         return 1 + traverse_up_parent(node)
    #     cost = traverse_up_parent(node)
    #     return cost

   # def search(self): #UNSURE WHAT SEARCH IS FOR
    #     # Implement the RRT* search algorithm here based on the project's requirements
    #     for i in range(self.max_iter):
    #         # Main RRT* logic goes here
    #         pass

    #     # Return the path found (if any) as a list of points [(x1, y1), (x2, y2), ...]
    #     return []
    

    #is rewire basically the shortening of the path??
    # def rewire(self, nodes, new_node, radius=1.5):
    #     for node in nodes:
    #         if node != new_node.parent and self.distance(node.point, new_node.point) <= radius and new_node.cost + self.distance(new_node.point, node.point) < node.cost:
    #             node.parent = new_node
    #             node.cost = new_node.cost + self.distance(new_node.point, node.point)