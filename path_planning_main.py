import pygame
import sys
from path_utils import RaceTrack
from rrt_star import rrt_star_planner
from rrt import rrt_planner

def main():
    pygame.init()

    SCREEN_WIDTH = 800
    SCREEN_HEIGHT = 800
    POINT_RADIUS = 5
    BACKGROUND_COLOR = (255, 255, 255)
    POINT_COLOR = (255, 0, 0)
    LINE_COLOR = (0, 0, 255)
    ROAD_THICKNESS = 50

    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Path Planning Simulator")

    racetrack = RaceTrack(SCREEN_WIDTH, SCREEN_HEIGHT, POINT_RADIUS, screen, POINT_COLOR=POINT_COLOR, LINE_COLOR=LINE_COLOR, ROAD_THICKNESS=ROAD_THICKNESS, seed=42)

    planner_path = []
    planner_selected = None  # Variable to keep track of which planner is selected

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1:
                    planner_selected = rrt_star_planner  # Select RRT* Planner
                elif event.key == pygame.K_2:
                    planner_selected = rrt_planner  # Select RRT Planner
                elif event.key == pygame.K_SPACE:
                    if planner_selected:  # Check if a planner has been selected
                        midpoints = racetrack.generate_race_course_midpath(20)
                        blue_cones, yellow_cones = racetrack.generate_left_and_right_cones()
                        planner_path = planner_selected(racetrack, blue_cones, yellow_cones, screen, POINT_RADIUS, SCREEN_WIDTH, SCREEN_HEIGHT)
        
        screen.fill(BACKGROUND_COLOR)
        racetrack.draw_yellow_cones()
        racetrack.draw_blue_cones()
        racetrack.draw_start()
        racetrack.draw_start_directon()
        
        if planner_path:  # Draw the path if it exists
            racetrack.draw_path(planner_path, [0]*len(planner_path))
        racetrack.flip()

    pygame.quit()
    sys.exit()

if __name__ == '__main__':
    main()
