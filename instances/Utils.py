import sys, os
import math
import random
import numpy as np
from typing import List, Dict, Tuple
from instances.Route import RouteObject
from instances.Trucks import Vehicle


class Instance:
    """
    n : int
        number of nodes in total
    Q : List
        list of trucks
    q : List[int]
        list of customer demands in kgs
    volumes : List[int]
        list of customer demands in volume
    customerDurations: List[float]
        list of customer durations (.nodes file)
    d : Dict[Tuple[int, int], float]
        list of total distances
    dInside : Dict[Tuple[int, int], float]
        list of distances inside
    dOutside : Dict[Tuple[int, int], float]
        list of distances outside
    arcDurations : Dict[Tuple[int, int], float]
        list of arc durations (.routes file)
    coordinates : List[Tuple[int, int]]
        list of customer coordinates
    """

    n: int
    Q: List
    q: List[int]
    volumes: List[int]
    customerDurations: List[float]
    d: Dict[Tuple, float]
    dInside: Dict[Tuple[int, int], float]
    dOutside: Dict[Tuple[int, int], float]
    arcDurations: Dict[Tuple[int, int], float]
    coordinates: List[Tuple[int, int]]

    def __init__(self, n: int, Q: List, q: List[int], volumes: List[int], customerDurations: List[float],
                 d: Dict[Tuple[int, int], float], dInside: Dict[Tuple[int, int], float],
                 dOutside: Dict[Tuple[int, int], float], arcDurations: Dict[Tuple[int, int], float],
                 coordinates: List[Tuple[int, int]]):
        self.n = n
        self.Q = Q
        self.q = q
        self.volumes = volumes
        self.customerDurations = customerDurations
        self.d = d
        self.dInside = dInside
        self.dOutside = dOutside
        self.arcDurations = arcDurations
        self.coordinates = coordinates

        # algorithm will run until first of these conditions is met. Either iterations or time
        self.max_iterations = 100 * 1
        self.max_time = 10e10
        # seconds

        """ The idea here is to fall back to our best known solution after getting away from it with SimAnnealing. 
        We need to allow enough iterations for the accepted solution to be optimized enough to compete with the bestSolution
        """

        self.max_iterations_no_improvement = max(50.0, self.max_iterations * 0.05)

        self.init_temp = 0.5  # factor with which the solution of the 0. iteration is turned into first temperature -> ourAlgorithm()
        temp_target_percentage = 0.025  # this parameter decides which % of the initial temperature should be achieved in the target iteration
        temp_target_iteration = 1.2  # this parameter decides in which iteration the target percentage should be used. iteration = 1/x77: 4 -> 25% of max iterations. 2 -> 50% of max iterations. 1.333 -> 75% of max iterations. 1 -> 100% of max iterations
        # example: with a temp_target_percentage of 0.01 and a temp_target_iteration of 2 we reach 1% of the initial temperature after 50% of the max iterations
        cooling_target = np.power(temp_target_percentage, (temp_target_iteration / self.max_iterations))  # this function sets our cooling factor dependent on the max_iterations. Example: (0.05, (2/self.max_iterations)) forces the temperature to 5% of the starting temp after 50% of iterations
        self.cooling = cooling_target  # factor with which temperature is reduced after every instance  -> simulated_annealing()
        self.freeze_period_length = 0.01  # currently set up to freeze for x * iterations. Not max_iterations, but iterations-so-far. So a freeze will be longer if the algorithm runs long
        self.final_effort = 0.02  # this parameter determines when we start our final effort. For the last x% of max_iterations, we will jump back to the best known solution and turn off simulated annealing. We then try to optimize this solution locally
        # set the initial weights for each operator
        self.init_weight_destroy_random = 25
        self.init_weight_destroy_expensive = 100
        self.init_weight_destroy_route = 50
        self.init_weight_destroy_related = 50
        self.init_weight_insert_cheapest = 50
        self.init_weight_insert_regret = 25

        # set upper and lower bounds for the number of destroyed nodes in destroy operations
        self.destroy_random_lb = 0.05  # of all customers. So if 112 customers & lb = 0.05: minimum 6
        self.destroy_random_ub = 0.15  # of all customers. So if 112 customers & ub = 0.15: maximum 17
        self.destroy_expensive_lb = 0.05
        self.destroy_expensive_ub = 0.1
        self.destroy_route_lb = 0.25  # of the chosen route
        self.destroy_route_ub = 1  # of the chosen route
        self.destroy_related_lb = 0.05
        self.destroy_related_ub = 0.15

        self.init_penalty = 10  # starting penalty costs in the 0. iteration -> penalty_cost()
        self.step_penalty = 0.1  # step by which penalty grows in every iteration -> penalty_cost()

        self.penalty_cost_iteration_for_initialization = 0.75 * self.max_iterations


Route = List[int]
Solution = List[Route]


def compute_distances_objects(solution: Solution, instance: Instance) -> float:
    sum_distances = 0.0

    for route in solution:
        sum_distances += compute_distance(route.customer_list, instance)

    return sum_distances


def compute_distance(route: Route, instance: Instance) -> float:
    sum_distances = 0.0

    # route: [0,1,2,3,4,0]
    #   (i-1)-^ ^-i
    for i in range(1, len(route)):
        key = (route[i - 1], route[i])
        sum_distances += instance.d[key]

    return sum_distances


def compute_distance_inside(route: Route, instance: Instance) -> float:
    sum_distances_ins = 0.0
    # route: [0,1,2,3,4,0]
    #   (i-1)-^ ^-i
    for i in range(1, len(route)):
        key = (route[i - 1], route[i])
        sum_distances_ins += instance.dInside[key]
    return sum_distances_ins


def compute_distance_outside(route: Route, instance: Instance) -> float:
    sum_distances_out = 0.0
    # route: [0,1,2,3,4,0]
    #   (i-1)-^ ^-i
    for i in range(1, len(route)):
        key = (route[i - 1], route[i])
        sum_distances_out += instance.dOutside[key]
    return sum_distances_out


def compute_total_demand(route: List[int], instance: Instance) -> int:
    sum_demands = 0
    for n in route:
        sum_demands += instance.q[n]

    return sum_demands


def compute_total_volume(route: List[int], instance: Instance) -> int:
    sum_volume = 0
    for n in route:
        sum_volume += instance.volumes[n]

    return sum_volume


def compute_duration(route: List[int], instance: Instance) -> float:
    sum_duration = 0.0

    # route: [0,1,2,3,4,0]
    #   (i-1)-^ ^-i
    for i in range(1, len(route)):
        key = (route[i - 1], route[i])
        sum_duration += instance.arcDurations[key]
        sum_duration += instance.customerDurations[route[i]]

    return sum_duration


def solution_cost(listOfRoutes: List[Route], instance: Instance, iteration: int, penalty_active=True) -> float:
    solutionCost = 0
    for r in listOfRoutes:
        solutionCost = solutionCost + routeCost(r, instance, iteration, penalty_active)
    return solutionCost


def routeCost(routeObject: RouteObject, instance: Instance, iteration: int, penalty_active=True) -> float:
    # distance
    distance_ins = compute_distance_inside(routeObject.customer_list, instance)
    cost_ins = distance_ins * routeObject.vehicle.cost_km_in
    distance_out = compute_distance_outside(routeObject.customer_list, instance)
    cost_out = distance_out * routeObject.vehicle.cost_km
    cost_km = cost_ins + cost_out
    distance = distance_ins + distance_out

    # duration
    duration = compute_duration(routeObject.customer_list, instance)
    cost_minute = duration * routeObject.vehicle.cost_m

    # fixed cost
    fixed_cost = routeObject.vehicle.fixed_cost

    # penalty
    cost_penalty = 0
    if penalty_active:  # checks if penalty_costs were enabled when calling the cost function
        cost_penalty = penalty_cost(routeObject, routeObject.vehicle, instance, iteration, distance, duration)

    cost = cost_km + cost_minute + cost_penalty + fixed_cost
    return cost


def penalty_cost(route: RouteObject, vehicle: Vehicle, instance: Instance, iteration: int, distance: float, duration: float) -> float:
    iteration_penalty = instance.init_penalty + iteration * instance.step_penalty  # penalty in each iteration.
    customer_list = route.customer_list

    # payload_kg
    constraint_kg = vehicle.payload_kg
    load_kg = compute_total_demand(customer_list, instance)
    if load_kg <= constraint_kg:
        penalty_kg = 0
    else:
        overload_kg = compute_overload(constraint_kg, load_kg)
        penalty_kg = overload_kg * iteration_penalty

    # payload_vol
    constraint_vol = vehicle.payload_vol
    load_vol = compute_total_volume(customer_list, instance)
    if load_vol <= constraint_vol:
        penalty_vol = 0
    else:
        overload_vol = compute_overload(constraint_vol, load_vol)
        penalty_vol = overload_vol * iteration_penalty

    # range
    constraint_range = vehicle.range_km
    if distance <= constraint_range:
        penalty_range = 0
    else:
        overload_range = compute_overload(constraint_range, distance)
        penalty_range = overload_range * iteration_penalty

    # duration
    constraint_duration = vehicle.max_duration
    if duration <= constraint_duration:
        penalty_duration = 0
    else:
        overload_duration = compute_overload(constraint_duration, duration)
        penalty_duration = overload_duration * iteration_penalty

    # combine
    penalty = penalty_kg + penalty_vol + penalty_range + penalty_duration  # + all other penalized constraints

    if penalty > 0:  # set the feasibility of the route by checking if penality > 0
        route.currently_feasible = False
    else:
        route.currently_feasible = True

    return penalty


def temporaryRouteCost(customer_list: Route, vehicle: Vehicle, instance: Instance, iteration: int, penalty_active=True) -> float:
    tempRoute = RouteObject(customer_list, vehicle)
    cost = routeCost(tempRoute, instance, iteration, penalty_active)
    return cost


def compute_overload(constraint: int, load: int) -> float:
    # Overload factor can be changed to our will
    overload_factor = (max(load - constraint, 0) / constraint)  # ** 2  # we normalize the factor by the constraint (to not punish more because values are higher), then we square to punish bigger overloads much more
    return overload_factor


def delete_empty_routes(list_of_routes: list[Route]) -> list[Route]:
    no_empty_routes = list_of_routes.copy()
    for r in list_of_routes:
        if len(r.customer_list) < 3:
            no_empty_routes.remove(r)
    return no_empty_routes


def simulated_annealing(instance: Instance, currentSolution: List[RouteObject], newSoluion: List[RouteObject], temp: float, iteration: int, freeze_period: int) -> bool:
    # https://github.com/perrygeo/simanneal
    curcost = solution_cost(currentSolution, instance, iteration, True)
    newcost = solution_cost(newSoluion, instance, iteration, True)
    accept = False
    rand = random.random()

    if newcost < curcost:
        accept = True
        print(f"We accepted a strictly better solution.")
    else:
        if freeze_period > 0:
            print(f"We reject the latest solution. We are in a freeze period for {freeze_period} iterations.")
        elif temp < 0.1:
            print(f"We reject the latest solution. Temperature {temp:.2f} was to cold for SimAnn.")
        else:
            if rand < math.exp((curcost - newcost) / temp):
                accept = True
                print(f"We accept the latest solution. We had a {100 * math.exp((curcost - newcost) / temp):.2f}% chance to accept.")
            else:
                print(f"We reject the latest solution. We had a {100 * math.exp((curcost - newcost) / temp):.2f}% chance to accept.")

    temperature = instance.cooling * temp  # update temperature
    freeze_period += -1
    return accept, newcost, temperature, freeze_period


# Disable printing
def blockPrint():
    sys.stdout = open(os.devnull, 'w')


# Restore printing
def enablePrint():
    sys.stdout = sys.__stdout__


def find_cheapest_vehicle(customer_list: list[int], instance: Instance, iteration: int, list_of_avb_vehicles: list[Vehicle]):
    distance_ins = compute_distance_inside(customer_list, instance)
    distance_out = compute_distance_outside(customer_list, instance)
    distance = distance_ins + distance_out
    duration = compute_duration(customer_list, instance)
    load_kg = compute_total_demand(customer_list, instance)
    load_vol = compute_total_volume(customer_list, instance)

    iteration_penalty = instance.init_penalty + iteration * instance.step_penalty

    best_vehicle = Vehicle("DummyType", "Paris", "none", "999999", True, True)
    best_vehicle_cost = 10e10
    last_vehicle_type = "FirstRound"
    for vehicle in list_of_avb_vehicles:
        if vehicle.type != last_vehicle_type:
            last_vehicle_type = vehicle.type

            cost_ins = distance_ins * vehicle.cost_km_in
            cost_out = distance_out * vehicle.cost_km
            cost_minute = duration * vehicle.cost_m

            constraint_kg = vehicle.payload_kg
            overload_kg = compute_overload(constraint_kg, load_kg)
            penalty_kg = overload_kg * iteration_penalty
            constraint_vol = vehicle.payload_vol
            overload_vol = compute_overload(constraint_vol, load_vol)
            penalty_vol = overload_vol * iteration_penalty
            constraint_range = vehicle.range_km
            overload_range = compute_overload(constraint_range, distance)
            penalty_range = overload_range * iteration_penalty
            constraint_duration = vehicle.max_duration
            overload_duration = compute_overload(constraint_duration, duration)
            penalty_duration = overload_duration * iteration_penalty

            fixed_cost = vehicle.fixed_cost

            this_vehicle_cost = cost_ins + cost_out + cost_minute + penalty_kg + penalty_vol + penalty_range + penalty_duration + fixed_cost

            if this_vehicle_cost < best_vehicle_cost:
                best_vehicle_cost = this_vehicle_cost
                best_vehicle = vehicle

    return best_vehicle_cost, best_vehicle


def vehicle_assignment(list_of_routes: list[Route], initial_list_of_vehicles: List[Vehicle], instance: Instance, iteration: int, penalty_active=True) -> List[Vehicle]:
    list_of_available_vehicles = initial_list_of_vehicles.copy()
    counter = 0
    for r in list_of_routes:  # check all routes. Before this they should be ordered by their costs descending
        counter += 1
        route_cost, best_vehicle = find_cheapest_vehicle(r.customer_list, instance, iteration, list_of_available_vehicles)
        list_of_available_vehicles.remove(best_vehicle)  # remove the bestVehicle from available
        r.vehicle = best_vehicle
        r.current_cost = routeCost(r, instance, iteration, penalty_active)  # update the route cost
        print(f"After Vehicle Assignment: route {counter} , vehicle {r.vehicle.plateNr}, cost: {r.current_cost:.2f} €, load: {compute_total_demand(r.customer_list, instance)} kg, vol: {compute_total_volume(r.customer_list, instance) / 1000:.2f} m^3, dist: {compute_distance(r.customer_list, instance):.0f} km, duration: {compute_duration(r.customer_list, instance):.0f} minutes, customerCount: {len(r.customer_list) - 2}, feasible: {r.currently_feasible}, customers: {r.customer_list}")
    return list_of_available_vehicles


class Instance_tune:
    """
    n : int
        number of nodes in total
    Q : List
        list of trucks
    q : List[int]
        list of customer demands in kgs
    volumes : List[int]
        list of customer demands in volume
    customerDurations: List[float]
        list of customer durations (.nodes file)
    d : Dict[Tuple[int, int], float]
        list of total distances
    dInside : Dict[Tuple[int, int], float]
        list of distances inside
    dOutside : Dict[Tuple[int, int], float]
        list of distances outside
    arcDurations : Dict[Tuple[int, int], float]
        list of arc durations (.routes file)
    coordinates : List[Tuple[int, int]]
        list of customer coordinates
    """

    n: int
    Q: List
    q: List[int]
    volumes: List[int]
    customerDurations: List[float]
    d: Dict[Tuple, float]
    dInside: Dict[Tuple[int, int], float]
    dOutside: Dict[Tuple[int, int], float]
    arcDurations: Dict[Tuple[int, int], float]
    coordinates: List[Tuple[int, int]]

    def __init__(self, n: int, Q: List, q: List[int], volumes: List[int], customerDurations: List[float],
                 d: Dict[Tuple[int, int], float], dInside: Dict[Tuple[int, int], float],
                 dOutside: Dict[Tuple[int, int], float], arcDurations: Dict[Tuple[int, int], float],
                 coordinates: List[Tuple[int, int]], args):
        self.n = n
        self.Q = Q
        self.q = q
        self.volumes = volumes
        self.customerDurations = customerDurations
        self.d = d
        self.dInside = dInside
        self.dOutside = dOutside
        self.arcDurations = arcDurations
        self.coordinates = coordinates

        # algorithm will run until first of these conditions is met. Either iterations or time
        self.max_iterations = args.max_iterations
        self.max_time = 10e10
        # seconds

        """ The idea here is to fall back to our best known solution after getting away from it with SimAnnealing. 
        We need to allow enough iterations for the accepted solution to be optimized enough to compete with the bestSolution
        """
        self.max_iterations_no_improvement = max(50, self.max_iterations * 0.05)

        self.init_temp = args.init_temp  # factor with which the solution of the 0. iteration is turned into first temperature -> ourAlgorithm()

        temp_target_percentage = args.temp_target_percentage  # this parameter decides which % of the initial temperature should be achieved in the target iteration
        temp_target_iteration = args.temp_target_iteration  # this parameter decides in which iteration the target percentage should be used. iteration = 1/x77: 4 -> 25% of max iterations. 2 -> 50% of max iterations. 1.333 -> 75% of max iterations. 1 -> 100% of max iterations.
        # example: with a temp_target_percentage of 0.01 and a temp_target_iteration of 2 we reach 1% of the initial temperature after 50% of the max iterations
        cooling_target = np.power(temp_target_percentage, (
                    temp_target_iteration / self.max_iterations))  # this function sets our cooling factor dependent on the max_iterations. Example: (0.05, (2/self.max_iterations)) forces the temperature to 5% of the starting temp after 50% of iterations.

        self.cooling = cooling_target  # factor with which temperature is reduced after every instance  -> simulated_annealing()
        self.freeze_period_length = args.freeze_period_length  # currently set up to freeze for x * iterations. Not max_iterations, but iterations-so-far. So a freeze will be longer if the algorithm runs long.

        self.final_effort = 0.02  # this parameter determines when we start our final effort. For the last x% of max_iterations, we will jump back to the best known solution and turn off simulated annealing. We then try to optimize this solution locally.

        # set the initial weights for each operator
        self.init_weight_destroy_random = 25
        self.init_weight_destroy_expensive = 100
        self.init_weight_destroy_route = 50
        self.init_weight_destroy_related = 50
        self.init_weight_insert_cheapest = 50
        self.init_weight_insert_regret = 25

        # set upper and lower bounds for the number of destroyed nodes in destroy operations
        self.destroy_random_lb = 0.05  # of all customers. So if 112 customers & lb = 0.05: minimum 6
        self.destroy_random_ub = args.destroy_random_ub  # of all customers. So if 112 customers & ub = 0.15: maximum 17
        self.destroy_expensive_lb = 0.05
        self.destroy_expensive_ub = args.destroy_expensive_ub
        self.destroy_route_lb = 0.25  # of the chosen route
        self.destroy_route_ub = args.destroy_route_ub  # of the chosen route
        self.destroy_related_lb = 0.05
        self.destroy_related_ub = args.destroy_related_ub

        # set max and min weights a ops can take
        self.max_weight = args.max_weight
        self.min_weight = args.min_weight
        self.reduce_step = args.reduce_step
        # self.increase_step = CURRENTLY SET TO ITERATION

        self.init_penalty = 25  # starting penalty costs in the 0. iteration -> penalty_cost()
        self.step_penalty = args.step_penalty  # step by which penalty grows in every iteration -> penalty_cost()

        # must be forced high enough not create ultra-infeasible solutions for short runs. Otherwise we get super long routes which cause very timeconsuming 2opt
        self.penalty_cost_iteration_for_initialization = max(1000, 0.75 * self.max_iterations)
