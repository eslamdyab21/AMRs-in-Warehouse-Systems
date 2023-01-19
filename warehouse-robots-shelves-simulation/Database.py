class Database():
	"""
	Write and query updated data from the database

	:param robots: (list) list of robots objects in the warehouse [robot1, robot2,....., robotn]
	:param shelves: (list) list of shelves objects in the warehouse [shelf1, shelf2,....., shelfn]
	:param map_size: (list) [map_size_x, map_size_y]
	"""

	def __init__(self, robots, shelvs, map_size):

	    self.robots = robots
	    self.shelvs = shelvs
	    self.map_size = map_size
	    self.min_cost = map_size[0]
	    self.robots_with_min_cost_list = []


	def query_from_db(self):
		self.shelvs[0].recived_order_status = True
		self.shelvs[1].recived_order_status = True



	def write_to_db(self):
		pass