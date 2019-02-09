
from maze import *
import math as m

import numpy as np

class CarState:
	def __init__(self, maze):
		self.maze = maze

		self.startNode = maze.start_node
		self.targetNode = maze.end_node

		self.cNode = self.startNode

	def calcHeuristic(self, nextNode):

		targetPntTuple = np.array(self.targetNode.coord.get_tuple())
		nextNodeTuple = np.array(nextNode.coord.get_tuple())

		dist = np.linalg.norm(targetPntTuple - nextNodeTuple)

		return dist


	def solve_maze(self):
		
		while 1:
			currentWays = self.cNode.dirNeighbours

			waysIndices = [i for i, j in enumerate(currentWays) if j is not None]

			waysCnt = len(waysIndices)
			if waysCnt == 1:
				self.tNode = currentWays[waysIndices[0]]
				print('Found only next node: ')
				print(self.tNode.show_info())
			else:
				print('There are multiple ways')
				for way in currentWays:
					if way is None:
						continue

					h = self.calcHeuristic(way)
					print('h = {}'.format(h))

				break


			self.cNode = self.tNode

