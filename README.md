# Autonomous Driving Planning and Control Project

## Overview
This project focuses on developing a path planning and control system for autonomous driving within a simulated racing environment. Utilizing algorithms such as **Rapidly-exploring Random Trees (RRT)** and its optimized version, **RRT***, the project aims to navigate a vehicle through complex tracks with efficiency and precision. The motivation behind choosing these algorithms is their effectiveness in exploring uncharted spaces quickly and optimizing paths in environments with unpredictable obstacles, making them ideal for the dynamic conditions of a race track.

<br>

## Visualization: RRT vs RRT*
### RRT
Course 1:

https://github.com/NathanielIskandar/Autonomous-Driving-Planning-and-Control-Project/assets/76016696/3602673b-1d3f-425c-8f59-ade0339f9558

Course 2:

https://github.com/NathanielIskandar/Autonomous-Driving-Planning-and-Control-Project/assets/76016696/c065200b-8a27-445d-b07e-ee4f8b7e8172



### RRT*
Note: Notice how RRT* performs path compression!

Course 1:

https://github.com/NathanielIskandar/Autonomous-Driving-Planning-and-Control-Project/assets/76016696/588a698a-fe39-48b6-99cc-c629136eb9b5



Course 2:

https://github.com/NathanielIskandar/Autonomous-Driving-Planning-and-Control-Project/assets/76016696/a4be40c9-3052-4728-a7be-8d82ef98ca01

For more videos, check out the following [Google Drive Link](https://drive.google.com/drive/folders/1f4mVHUy-8IKJi7Qxet3e6x41tSt3d4xa?usp=drive_link).


<br>


## A Closer Look Into The Algorithms
### Rapidly-exploring Random Trees (RRT)
Definition: Rapidly-exploring Random Trees (RRT) is a path planning algorithm designed for efficiently searching non-convex, high-dimensional spaces by randomly building a space-filling tree. The algorithm selects random points in the search space and extends the tree from its nearest vertex towards these points. This process is repeated until the tree reaches the goal region or a specified number of iterations is completed.

Algorithm: RRT operates by initializing a tree with a root node at the start position. At each iteration, a point is randomly sampled, and the nearest tree node is extended towards this point by a predefined step size. If this new node does not collide with any obstacle, it is added to the tree. The algorithm terminates when a node within a specific threshold distance from the goal is added to the tree or after a predetermined number of iterations.

### RRT* (Optimized Rapidly-exploring Random Trees)
Definition: RRT*, introduced by Dr. Sertac Karaman and Dr. Emilio Frazzoli, enhances the RRT algorithm by ensuring that as more nodes are added, the path converges towards the optimal solution. RRT* iteratively refines the tree by checking if paths to new nodes can be shortened through other nodes and by rewiring the tree's connections to minimize path costs.

Algorithm: Similar to RRT, RRT* begins with a tree rooted at the start position. When a new point is sampled and added to the tree, RRT* additionally searches for neighboring nodes within a defined radius. If a cheaper path to any of these neighbors exists through the new node, the tree is rewired to reflect this shorter path. This process not only expands the tree towards the goal but also optimizes the paths within the tree over time.

### Research Papers:
1. RRT: "Rapidly-exploring random trees: A new tool for path planning" by Steven M. LaValle. [Read here](https://msl.cs.illinois.edu/~lavalle/papers/Lav98c.pdf).
2. RRT*: "Sampling-based algorithms for optimal motion planning" by Sertac Karaman and Emilio Frazzoli. [Read here](https://people.eecs.berkeley.edu/~pabbeel/cs287-fa19/optreadings/rrtstar.pdf).


<br>


## Features
1. **Rapidly-exploring Random Trees (RRT)**: Implements the RRT algorithm to explore available paths in a randomly generated racing environment.
   - RRT Optimization: Enhances the RRT algorithm by continuously refining the path to find the most efficient route to the goal.
2. **Simulation Environment**: Utilizes pygame for visualizing the path planning process in a simulated racing environment.
3. **Obstacle Avoidance**: Integrates obstacle detection and avoidance into the path planning process using shapely for geometric calculations.
4. **Efficient Path Planning**: Demonstrates the capability to generate direct and efficient routes through cluttered spaces.


<br>


## Technologies
1. Python 3.x
2. Pygame for simulation visualization
3. Numpy for numerical calculations
4. Shapely for geometric operations

<br>

## Installation
To run this project locally, you need to have Python installed on your machine. After cloning the repository, navigate to the project directory and install the required dependencies:
```
pip install numpy shapely pygame
```

<br>


## Usage
To start the simulation, run the main script from the terminal:
```
python path_planning_main.py
```

To generate a new course and visualize the path planning process using  the **RRT algorithm**:
1. Press the number one (1)
2. Press the space bar

To generate a new course and visualize the path planning process using  the **RRT\* algorithm**:
1. Press the number two (2)
2. Press the space bar


<br>


## Future Directions
Moving forward, the project aims to delve deeper into optimization algorithms, specifically leveraging the `cvxpy` library to perform complex computational math for optimization techniques as taught in EECS127. This direction involves a more profound understanding of mathematical optimization problems and integrating these concepts to enhance the autonomous driving system's efficiency and adaptability.
