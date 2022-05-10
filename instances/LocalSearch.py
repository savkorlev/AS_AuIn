from instances.Route import RouteObject
from instances.Trucks import Vehicle
from instances.Utils import compute_distance, Solution, Instance, compute_total_demand, find_cheapest_vehicle, \
    delete_empty_routes, compute_duration


def hillclimbing(solution: Solution, instance: Instance, function) -> Solution:
    """
    simple improvement procedure which tries to find a (local) optima by continuously calling
    find_first_improvement_2Opt(solution, instance) until no further improvements are found.
    :param solution: list of routes to improve
    :param instance: corresponding instance
    :return: the provided solution or an improved solution
    """
    improvement = True
    while improvement:  # until no improvement, 2 opt will continue
        improvement = function(solution, instance)

    return solution


def vnd(solution: Solution, instance: Instance) -> Solution:
    """
    Variable Neighborhood Decent procedure, searching three neighborhoods (2opt, relocate, exchange) in a structured
    way. Neighborhoods are ordered, if an improvement was found, the search continues from the first neighborhood.
    Otherwise the next neighborhood in order is searched. This continues until no improvements can be found.
    :param solution: list of routes to improve
    :param instance: corresponding instance
    :return: the provided solution or an improved solution
    """
    improvement = True
    while improvement:
        improvement = find_first_improvement_2Opt(solution, instance)
        if not improvement:  # no improvement from 2Opt -> relocate
            improvement = find_first_improvement_relocate(solution, instance)
    return solution


def find_first_improvement_2Opt(solution: Solution, instance: Instance) -> bool:
    """
    search for the first improving 2-opt move.
    A 2-opt consist of removing two edges and reconnecting the nodes differently but feasible way.
    In other terms, a consecutive subset of visits is reversed.
    Example: [0,1,2,3,4,5,0] => [0,1,4,3,2,5,0]
        A 2-opt could be removing edges between 1-2 and 4-5, and connecting 1-4 and 2-5.
    :param solution: list of routes to improve
    :param instance: corresponding instance
    :return: `True` if an improvement was found, otherwise `False`.
    """

    for r_index, route in enumerate(solution):
        for i in range(1, len(route) - 2):  # i: the first position to be switched (the last index should -2)
            for j in range(i + 1, len(route) - 1):  # j: the second position to be switched (the last index should -1)
                # [0,1,2,3,4,5,0]
                #     i^---^j
                # [0,1,4,3,2,5,0]
                # the route until (i-1) + reverse (i~j) + the route from (j+1)
                new_route = route[:i] + list(reversed(route[i:(j + 1)])) + route[(j + 1):]
                new_distance = compute_distance(new_route, instance)

                change = new_distance - compute_distance(route, instance)

                if change < -0.000001:
                    # improvement
                    solution[r_index] = new_route
                    return True
    return False


def find_first_improvement_relocate(solution: Solution, instance: Instance) -> bool:
    """
    search for the first improving relocate
    Example: [[0,1,2,0],[0,3,4,5,0]] => [[0,1,3,2,0],[0,4,5,0]]
        A relocate move could be to move customer 3 between 1 and 2.
    :param solution: list of routes to improve
    :param instance: corresponding instance
    :return: `True` if an improvement was found, otherwise `False`.
    """

    for r1_index, r1 in enumerate(solution):
        for i in range(1, len(r1) - 1):  # except depot, all the nodes can be relocated
            for r2_index, r2 in enumerate(solution):
                # demand of node to be relocated + total demands of route to have the relocated node > max capacity
                if instance.q[r1[i]] + compute_total_demand(r2, instance) > 2400:
                    continue

                for j in range(1, len(r2)):  # relocate the node to every position of the other route
                    if r1_index == r2_index:  # same route
                        continue

                    new_r1 = r1[:i] + r1[i + 1:]  # node i is removed
                    new_r2 = r2[:j] + [r1[i]] + r2[j:]  # [r1[i]] is the relocated node

                    change_r1 = compute_distance(new_r1, instance) - compute_distance(r1, instance)
                    change_r2 = compute_distance(new_r2, instance) - compute_distance(r2, instance)

                    if change_r1 + change_r2 < -0.000001:  # if overall distance becomes smaller
                        solution[r1_index] = new_r1
                        solution[r2_index] = new_r2
                        return True
    return False


def find_first_improvement_exchange(solution: Solution, instance: Instance) -> bool:
    """
    search for the first improving exchange
    Example: [[0,1,2,0],[0,3,4,5,0]] => [[0,4,2,0],[0,3,1,5,0]]
        A exchange move could be to swap customer 4 between 1.
    :param solution: list of routes to improve
    :param instance: corresponding instance
    :return: `True` if an improvement was found, otherwise `False`.
    """

    current_distances = []
    for route in solution:
        current_distances.append(compute_distance(route, instance))

    current_demand = []
    for route in solution:
        current_demand.append(compute_total_demand(route, instance))

    for r1_index, r1 in enumerate(solution):
        for i in range(1, len(r1) - 1):  # except depot, all the nodes can be relocated


            for r2_index, r2 in enumerate(solution):
                if r1_index == r2_index:  # same route
                    continue

                for j in range(1, len(r2) - 1):
                    # demand of route 1 - demand of swapped node in route 1 + demand of swapped node in route 2
                    new_demand_r1 = current_demand[r1_index] - instance.q[r1[i]] + instance.q[r2[j]]
                    # demand of route 2 - demand of swapped node in route 2 + demand of swapped node in route 1
                    new_demand_r2 = current_demand[r2_index] - instance.q[r2[j]] + instance.q[r1[i]]

                    if new_demand_r1 > 2400 or new_demand_r2 > 2400:  # exceed max capacity
                        continue
                    change_r1 = - instance.d[r1[i - 1], r1[i]] \
                                - instance.d[r1[i], r1[i + 1]] \
                                + instance.d[r1[i - 1], r2[j]] \
                                + instance.d[r2[j], r1[i + 1]]
                    change_r2 = - instance.d[r2[j - 1], r2[j]] \
                                - instance.d[r2[j], r2[j + 1]] \
                                + instance.d[r2[j - 1], r1[i]] \
                                + instance.d[r1[i], r2[j + 1]]

                    if change_r1 + change_r2 < -0.000001:  # if overall distance becomes smaller
                        # only create the new route if it's an improvement
                        new_r1 = r1[:i] + [r2[j]] + r1[i + 1:]
                        new_r2 = r2[:j] + [r1[i]] + r2[j + 1:]

                        solution[r1_index] = new_r1
                        solution[r2_index] = new_r2
                        return True
    return False


def find_best_improvement_2Opt(solution: Solution, instance: Instance) -> bool:
    """
    search for the best improving 2-opt move.
    :param solution: list of routes to improve
    :param instance: corresponding instance
    :return: `True` if an improvement was found, otherwise `False`.
    """
    # https://github.com/saper0/tsp-heuristics/blob/master/ls_2opt.py

    for r_index, route in enumerate(solution):
        best_2opt = None
        best_distance = compute_distance(route, instance)
        for i in range(1, len(route) - 2):  # i: the first position to be switched (the last index should -2)
            for j in range(i + 1, len(route) - 1):  # j: the second position to be switched (the last index should -1)
                new_route = route[:i] + list(reversed(route[i:(j + 1)])) + route[(j + 1):]
                new_distance = compute_distance(new_route, instance)

                change = new_distance - best_distance

                if change < -0.000001:
                    # improvement
                    best_distance = new_distance
                    best_2opt = (i, j)
        if best_2opt is not None:
            solution[r_index] = route[:best_2opt[0]] + list(reversed(route[best_2opt[0]:(best_2opt[1] + 1)])) + route[(best_2opt[1] + 1):]
            return True
    return False


def find_best_duration_improvement_2Opt(solution: Solution, instance: Instance) -> bool:
    """
    search for the best improving 2-opt move.
    :param solution: list of routes to improve
    :param instance: corresponding instance
    :return: `True` if an improvement was found, otherwise `False`.
    """
    # https://github.com/saper0/tsp-heuristics/blob/master/ls_2opt.py

    for r_index, route in enumerate(solution):
        best_2opt = None
        best_duration = compute_duration(route, instance)
        for i in range(1, len(route) - 2):  # i: the first position to be switched (the last index should -2)
            for j in range(i + 1, len(route) - 1):  # j: the second position to be switched (the last index should -1)
                new_route = route[:i] + list(reversed(route[i:(j + 1)])) + route[(j + 1):]
                new_duration = compute_duration(new_route, instance)

                change = new_duration - best_duration

                if change < -0.000001:
                    # improvement
                    best_duration = new_duration
                    best_2opt = (i, j)
        if best_2opt is not None:
            solution[r_index] = route[:best_2opt[0]] + list(reversed(route[best_2opt[0]:(best_2opt[1] + 1)])) + route[(best_2opt[1] + 1):]
            return True
    return False


def combine_routes(list_of_routes: list[RouteObject], list_of_avb_vehicles: list[Vehicle], instance: Instance, iteration: int) -> list[RouteObject]:
    for position_a, route_a in enumerate(list_of_routes):
        if route_a.customer_list != [0, 0] and route_a.vehicle.type != "MercedesBenzAtego":  # don't try to combine with Atego-Routes

            for position_b, route_b in enumerate(list_of_routes):
                if route_b != route_a and route_b.vehicle.type != "MercedesBenzAtego":  # don't try to combine with Atego-Routes
                    if list_of_routes[position_a].customer_list != [0, 0] and list_of_routes[position_b].customer_list != [0, 0]:  # we dont try to combine empty lists

                        temp_list_of_avb_vehicles = list_of_avb_vehicles.copy()
                        temp_list_of_avb_vehicles.append(route_a.vehicle)  # add the vehicles of the two routes into the pool temporarely
                        temp_list_of_avb_vehicles.append(route_b.vehicle)

                        customer_list_a = []
                        for i in range(len(route_a.customer_list) - 2):  # turns [0, 1, 2, 3, 0] into [1, 2, 3]
                            customer_list_a.append(route_a.customer_list[i + 1])
                        customer_list_b = []
                        for j in range(len(route_b.customer_list) - 2):  # turns [0, 1, 2, 3, 0] into [1, 2, 3]
                            customer_list_b.append(route_b.customer_list[j + 1])

                        temp_customer_list_ab = [0] + customer_list_a + customer_list_b + [0]  # combining the two lists into one
                        cost_temp_ab, vehicle_temp_ab = find_cheapest_vehicle(temp_customer_list_ab, instance, iteration, temp_list_of_avb_vehicles)  # finding the cheapest vehicle for the new route

                        cost_2routes = route_a.current_cost + route_b.current_cost
                        if cost_temp_ab < cost_2routes:  # we operate with first improvement. As soon as we find a combined route cheaper than the two small routes we combine
                            new_route = RouteObject(temp_customer_list_ab, vehicle_temp_ab)
                            new_route.current_cost = cost_temp_ab
                            list_of_routes.append(new_route)
                            list_of_routes[position_a].customer_list = [0, 0]  # we set the old routes to empty. This way they will not be chosen in the next iterations of the for loop
                            list_of_routes[position_b].customer_list = [0, 0]

                            list_of_avb_vehicles.append(
                                route_a.vehicle)  # adding the vehicles of the closed routes back into the pool
                            list_of_avb_vehicles.append(route_b.vehicle)
                            list_of_avb_vehicles.remove(
                                vehicle_temp_ab)  # removing the chosen vehicle for the combined route from the pool
                            print(
                                f"We combined route {position_a + 1} and route {position_b + 1} into {new_route.customer_list}. Chosen Vehicle was {new_route.vehicle.type}")

    new_list_of_routes = delete_empty_routes(list_of_routes)
    return new_list_of_routes, list_of_avb_vehicles
