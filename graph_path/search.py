from maze import *
import pygame
from pygame.locals import *

import numpy as np

import collections

class Queue:
    def __init__(self):
        self.elements = collections.deque()
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, x):
        self.elements.append(x)
    
    def get(self):
        return self.elements.popleft()

class MazeSearch:
    def __init__(self, maze):
        self.maze = maze

    def get_path(self, start_node, start_src_idx, end_node):
        self.curr_node = start_node
        self.curr_src_idx = start_src_idx

        frontier = Queue()
        frontier.put( (self.curr_node, self.curr_src_idx) )

        come_from = {}
        come_from[self.curr_node] = None

        while not frontier.empty():
            self.curr_node, self.curr_src_idx = frontier.get()
            # print("Visiting {}".format(self.curr_node))

            if self.curr_node == end_node:
                break

            neighbours = self.curr_node.getTransNeighbours( self.curr_src_idx )
            # print('Show neighbours')
            # for nb in neighbours:
                # print(nb)

            for next_nd in neighbours:
                if next_nd is not None and next_nd not in come_from:
                    next_nd_src_idx = next_nd.get_source_idx_node( self.curr_node )

                    frontier.put( (next_nd, next_nd_src_idx) )
                    
                    come_from[next_nd] = self.curr_node
        
        return come_from

        # neighbours = self.curr_node.getTransNeighbours( self.curr_src_idx )
        
        # print('Show neighbours')
        # for nb in neighbours:
            # print(nb)

if __name__ == '__main__':

    pygame.init()

    structure = [[0, 0, 0, 0, 8, 8, 0, 8, 8, 0, 8, 8],
                 [8, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [8, 0, 8, 8, 8, 8, 8, 0, 8, 8, 0, 8],
                 [8, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8],
                 [8, 0, 8, 8, 8, 0, 8, 8, 8, 8, 8, 8],
                 [8, 0, 8, 0, 8, 0, 8, 8, 8, 8, 8, 8],
                 [0, 0, 0, 0, 8, 0, 8, 8, 8, 8, 8, 8],
                 [0, 8, 8, 0, 8, 2, 8, 8, 8, 8, 8, 8],
                 [0, 8, 8, 0, 8, 0, 8, 8, 8, 8, 8, 8],
                 [0, 0, 0, 0, 8, 0, 8, 8, 8, 8, 8, 8],
                 [0, 8, 8, 0, 8, 0, 8, 8, 8, 8, 8, 8],
                 [0, 0, 0, 0, 0, 0, 8, 8, 8, 8, 8, 8]]
    structure = np.array(structure, np.uint8)

    my_maze     = Maze(structure)

    my_mz_state = MazeState(my_maze)
    search      = MazeSearch(my_maze)

    while True:
        # Show results
        come_from_dict = search.get_path(my_mz_state.curr_node, my_mz_state.curr_src_idx, my_mz_state.target_node)

        print('Result for {} {}'.format( my_mz_state.curr_node, my_mz_state.curr_src_idx ))
        
        curr_node = my_mz_state.target_node
        while 1:
            print('{} from {}'.format(curr_node, come_from_dict[curr_node]))
            curr_node = come_from_dict[curr_node]
            if curr_node is None:
                break

        # print(maze.maze_array)
        my_mz_state.render()

        next_move = False

        while True:
            if next_move:
                break

            for event in pygame.event.get():
                if event.type == KEYDOWN:
                    if event.key == pygame.K_LEFT:
                        print('Left')
                        if my_mz_state.makeMoveDir(MazeState.STATE_MOVE_LEFT):
                            next_move = True
                            break
                    if event.key == pygame.K_RIGHT:
                        print('Right')
                        if my_mz_state.makeMoveDir(MazeState.STATE_MOVE_RGHT):
                            next_move = True
                            break
                    if event.key == pygame.K_UP:
                        print('Up')
                        if my_mz_state.makeMoveDir(MazeState.STATE_MOVE_FRWD):
                            next_move = True
                            break
                    if event.key == pygame.K_DOWN:
                        print('Down')
                        exit(1)


    pygame.quit()


