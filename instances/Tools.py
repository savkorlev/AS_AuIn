from scipy.spatial.distance import cityblock
from instances.Nodes import Node


def create_graph(locations, demands):  # to create locations, assigning demands and distances

    location_list = []
    locations.append(locations[0])  # to create a dummy return depot (n+1)
    demands.append(demands[0])
    for i in range(len(locations)):
        location_list.append(Node(id="l_" + str(i), index=i, demand=demands[i]))  # creating the locations

    for i in location_list:
        for j in location_list:
            i.distance[j] = float(cityblock(locations[i.index], locations[j.index]))
            # cityblock function returns the manhattan distance between two vectors

    return location_list

def print_solution(cpx, location_list, x, y): # a function to evaluate and print the solution of a model
    cpx.solve()
    print()
    print("Solution status = ", cpx.solution.get_status(), ":", end=' ')
    print(cpx.solution.status[cpx.solution.get_status()])
    print("Solution value  = ", cpx.solution.get_objective_value())
    print()

    solution_x, solution_y = [], {}

    for i in location_list:
        for j in location_list:
            if j.index != i.index:
                if cpx.solution.get_values(x[i.index][j.index]) > 0:
                    solution_x.append([i.index, j.index, i.distance[j]])
                    # distances between the visited locations (with the order)
        solution_y[i.index] = round(cpx.solution.get_values(y[i.index]), 0)
        # values of the decision variables y (load of the vehicle)

    starter, follower = [], [] # find the starting locations of the routes
    for i in range(len(solution_x)):
        if solution_x[i][0] == 0:
            starter.append([solution_x[i][0], solution_x[i][1], solution_x[i][2]])
        else:
            follower.append([solution_x[i][0], solution_x[i][1], solution_x[i][2]])

    routes, route_length = [], [] # find the routes and their lengths
    for i in range(len(starter)):
        to_go = starter[i][1] # starting from a location which is first visited by a vehicle from the depot,
        current_route = [starter[i][0], to_go] # create a route,
        current_length = starter[i][2] # sum the distances between the traversed locations,
        while to_go != location_list[-1].index: # until the route is completed in the depot
            for j in range(len(follower)):
                if follower[j][0] == to_go:
                    current_route.append(follower[j][1])
                    current_length += follower[j][2]
                    to_go = follower[j][1]
        routes.extend([current_route])
        route_length.append(current_length)

    # print the routes of the vehicles and the total load of a vehicle after visiting a specific location
    for i in range(len(routes)):
        print("Route for vehicle " + str(i + 1))
        for j in range(len(routes[i]) - 1):
            print(str(routes[i][j]) + " load: (" + str(solution_y[routes[i][j]]) + ") --> ", end=' ')
        print("0") # 0 presents the return here
        print("Route length: " + str(route_length[i]))
        print()
