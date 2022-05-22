class Node:  # define a class to generate location objects
    def __init__(self, id, index, demand):  # constructor, initiates a location object
        self.id = id
        self.index = index
        self.distance = {}  # dictionary to keep the distance values to other nodes
        self.demand = demand  # demand of each location

    def __str__(self):  # if we try to print a vertex object, this function returns the id (string)
        return self.id
