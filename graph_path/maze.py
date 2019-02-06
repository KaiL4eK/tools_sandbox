import numpy as np
import itertools as it

import pygame
from pygame.locals import *

class MazeNode:
    def __init__(self):
        self.directions = [0, 0, 0, 0]



cell_colors = (255, 255, 255), (0, 255, 0), (128, 128, 255)
cell_margin = 2

class Point:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def get_tuple(self):
        return (self.x, self.y)

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)

class Node:

    node_idx_cntr = 0
    

    dir_deltas = [Point(0, 1),
                  Point(1, 0),
                  Point(0, -1),
                  Point(-1, 0)]

    TRANS_IDX_LEFT  = 0
    TRANS_IDX_FRWD  = 1
    TRANS_IDX_RGHT  = 2

    # [src_id][TRANS_IDX_*] = place in <next_nodes>
    translation_table = np.array( [
                                    [1, 2, 3],
                                    [2, 3, 0],
                                    [3, 0, 1],
                                    [0, 1, 2]
                                    ], dtype=np.uint8 )

    def __init__(self, node_coord):
        # Up, right, down, left
        self.directions = [0, 0, 0, 0]
        self.coord      = node_coord
        self.next_nodes = [None, None, None, None]

        self.idx = -1

    def __str__(self):
        return 'id: {}'.format(self.idx)

    def setMainNode(self):
        self.idx            = Node.node_idx_cntr
        Node.node_idx_cntr  += 1

    def getNextNode(self, trans_idx, curr_src_idx):
        trans_neighbours = self.getTransNeighbours(curr_src_idx)

        return trans_neighbours[trans_idx]

    # Only for MainNode
    def get_source_idx_node(self, src_node):
        for src_idx in range(4):
            if src_node == self.next_nodes[src_idx]:
                return src_idx

        return -1 # Error


    def get_source_idx_coord(self, src_coord):
        delta = src_coord - self.coord
        # print('Delta: {} - {}'.format(self.coord.get_tuple(), src_coord.get_tuple()))

        for src_idx, dlt in enumerate(Node.dir_deltas):
            if delta.x == dlt.x and delta.y == dlt.y:
                return src_idx

        # exit(1)
        return -1 # Error 

    def getTransNeighbours(self, src_idx):
        trans_neighbours = [None, None, None]

        tr_table = Node.translation_table[src_idx]

        for tr_idx, dir_idx in enumerate(tr_table):
            trans_neighbours[tr_idx] = self.next_nodes[dir_idx]

        return trans_neighbours

    # def edge_get_next_point(self, src_coord):
    #     src_dir_idx = self.get_source_idx_coord(src_coord)
    #     if src_dir_idx < 0:
    #         print('Failed get_source_idx()')
    #         exit( 1 )

    #     next_dir_idxs = [i for i, x in enumerate(self.directions) if i != src_dir_idx and x == 1]
    #     if len(next_dir_idxs) != 1:
    #         print('Achtung!')
    #         exit(1)

    #     next_dir_idx = next_dir_idxs[0]

    #     return self.coord + Node.dir_deltas[next_dir_idx]

class MazeState:
    
    STATE_MOVE_LEFT = 0
    STATE_MOVE_FRWD = 1
    STATE_MOVE_RGHT = 2

    def __init__(self, maze):
        self.maze = maze

        self.curr_node    = maze.start_node
        self.curr_src_idx = maze.start_src_idx
        self.target_node  = maze.end_node

    def makeMoveDir(self, state_move):

        trans_moves = {
            MazeState.STATE_MOVE_LEFT: Node.TRANS_IDX_LEFT,
            MazeState.STATE_MOVE_FRWD: Node.TRANS_IDX_FRWD,
            MazeState.STATE_MOVE_RGHT: Node.TRANS_IDX_RGHT
        }

        next_node = self.curr_node.getNextNode(trans_moves[state_move], self.curr_src_idx)
        if next_node is None:
            print('No such next node')
            return False

        next_nd_src_idx = next_node.get_source_idx_node(self.curr_node)

        self.curr_node    = next_node
        self.curr_src_idx = next_nd_src_idx

        return True


    def render_get_cell_rect(self, coordinates, screen):
        x, y = coordinates
        y = self.maze.height - 1 - y
        cell_width = screen.get_width() / self.maze.width
        adjusted_width = cell_width - cell_margin
        return pygame.Rect(x * cell_width + cell_margin / 2,
                           y * cell_width + cell_margin / 2,
                           adjusted_width, adjusted_width)


    def render(self):
        screen = pygame.display.set_mode((320, 320))
        screen.fill((0, 0, 0))
        
        font = pygame.font.Font(pygame.font.get_default_font(), 12)

        for key in self.maze.edges:
            screen.fill(cell_colors[0], self.render_get_cell_rect(key, screen))

        for key in self.maze.nodes:
            rect = self.render_get_cell_rect(key, screen)

            if self.maze.nodes[key] == self.maze.start_node or self.maze.nodes[key] == self.maze.end_node:
                screen.fill(cell_colors[2], rect)
            else:
                screen.fill(cell_colors[1], rect)

            # Draw current node
            if self.maze.nodes[key] == self.curr_node:
                screen.fill((128, 128, 128), rect)

            text = font.render("{}".format(self.maze.nodes[key].idx), True, (0, 0, 0))
            text_rect = text.get_rect()
            text_rect.center = rect.center

            screen.blit(text, text_rect)

        pygame.display.update()



class Maze:

    START_NODE_ID   = 1
    END_NODE_ID     = 2

    def __init__(self, structure):
        self.height, self.width = structure.shape
        self.idx_set = set(it.product(range(self.width), range(self.height)))
        self.maze_array = structure

        self.nodes = {}
        self.edges = {}

        for x in range(self.height):
            for y in range(self.width):
                point = Point(x, y)
                elem = self.get_maze_element(point)
                if self.is_element_vacant(elem):
                    node = Node(point)
                    print(node)

                    for i, delta in enumerate(Node.dir_deltas):
                        neighbour = self.get_maze_element(point + delta)
                        if neighbour is not None and self.is_element_vacant(neighbour):
                            node.directions[i] = 1

                    if elem == Maze.START_NODE_ID:
                        self.start_node = node
                        self.start_src_idx = 3   # Left
                        self.nodes[point.get_tuple()] = node
                        continue

                    if elem == Maze.END_NODE_ID:
                        self.end_node   = node
                        self.nodes[point.get_tuple()] = node
                        continue

                    if sum(node.directions) == 2:
                        self.edges[point.get_tuple()] = node
                    else:
                        self.nodes[point.get_tuple()] = node

        for key in self.nodes:
            curr_node  = self.nodes[key]

            print('Current node: {}'.format(key))
            
            for dir_idx, d in enumerate(curr_node.directions):
                curr_point = curr_node.coord
                if d == 1:
                    print('Direction: {}'.format(dir_idx))
                    next_point = curr_point + Node.dir_deltas[dir_idx]

                    while (1):
                        print('Next: {}'.format(next_point.get_tuple()))
                        if next_point.get_tuple() in self.nodes:
                            print('>>Node')
                            curr_node.next_nodes[dir_idx] = self.nodes[next_point.get_tuple()]
                            break;
                        elif next_point.get_tuple() in self.edges:
                            print('>>Edge')
                            edge = self.edges[next_point.get_tuple()]
                            prev_point = curr_point
                            curr_point = next_point


                            # Working with edge
                            src_dir_idx = edge.get_source_idx_coord(prev_point)
                            if src_dir_idx < 0:
                                print('Failed get_source_idx()')
                                exit( 1 )
                            next_dir_idxs = [i for i, x in enumerate(edge.directions) if i != src_dir_idx and x == 1]
                            if len(next_dir_idxs) != 1:
                                print('Achtung!')
                                exit(1)
                            next_dir_idx = next_dir_idxs[0]

                            next_point = edge.coord + Node.dir_deltas[next_dir_idx]



                            # next_point = edge.edge_get_next_point(prev_point)
                        else:
                            print('>>Failed')
                            return

        for key in self.nodes:
            self.nodes[key].setMainNode()

        for key in sorted(self.nodes, key=lambda key: key[0]):
            print('{}   \t{}'.format(key, self.nodes[key]))
            for node in self.nodes[key].next_nodes:
                print('\t{}'.format(node))


    def is_element_vacant(self, elem):
        if elem == 0 or elem == 1 or elem == 2:
            return True

        return False

    def get_maze_element(self, p):
        if p.x < 0 or p.x >= self.width:
            return None

        if p.y < 0 or p.y >= self.height:
            return None

        return self.maze_array[self.height-p.y-1][p.x];


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

    maze = Maze(structure)

    # print(maze.maze_array)
    maze.render_maze()

    while True:
        for event in pygame.event.get():
            if event.type == KEYDOWN:
                exit()


    pygame.quit()

