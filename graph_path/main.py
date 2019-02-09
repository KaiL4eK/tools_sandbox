import numpy as np
import pygame
from pygame.locals import *

from maze import *
from car_state import *

# local TF -> ROS: (x1, y1) = (2*y, -2*x)
# ROS -> local TF: (x, y) = (-y1/2, x1/2)

def main(args=None):

    pygame.init()

    structure = [[0, 0, 0, 0, 0, 0, 0],
                 [0, 8, 0, 8, 0, 8, 0],
                 [0, 8, 0, 8, 0, 0, 0],
                 [0, 0, 0, 0, 0, 8, 8],
                 [0, 8, 0, 8, 0, 8, 8],
                 [0, 8, 0, 8, 0, 0, 2],
                 [1, 8, 0, 0, 0, 8, 8]]
    structure = np.array(structure, np.uint8)
    maze = Maze(structure)

    maze.render_maze()
    maze.nextPreprocessing()

    car = CarState(maze)
    car.solve_maze()

    while True:
        for event in pygame.event.get():
            if event.type == KEYDOWN:
                if event.key == 32: # Space
                    exit()



    pygame.quit()


if __name__ == '__main__':
    main()
