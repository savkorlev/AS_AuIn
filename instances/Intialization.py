import copy
import math
import random
from instances.Route import RouteObject
from instances.Trucks import Vehicle
from instances.Utils import Instance, temporaryRouteCost


def next_fit_heuristic(all_customers: list[int], instance: Instance, initialVehicles: list[Vehicle]) -> list():

    """ penalty_cost_iteration_for_initialization sets the iteration that gives us the penalty cost for deciding if it is cheaper
    to add a new route or to add to the last open route.
    If this is chosen too high will only create feasible routes (if possible) and tends to overload the last vehicle available.
    Setting it too low only creates infeasible routes.
    A good idea seems to be to set it to 75% of maxIteration"""

    listOfRoutes = []
    availableVehicles = copy.deepcopy(initialVehicles)

    randomVehicle = random.choice(availableVehicles)
    listOfRoutes.append(RouteObject([0, 0], randomVehicle))
    availableVehicles.remove(randomVehicle)

    for current_customer in all_customers:
        last_route = listOfRoutes[len(listOfRoutes) - 1]
        last_position = len(last_route.customer_list) - 1

        temp_route = last_route.customer_list.copy()
        temp_route.insert(last_position, current_customer)
        # set iteration to something where it will already have reasonable penalties
        cost_with = temporaryRouteCost(temp_route, last_route.vehicle, instance, instance.penalty_cost_iteration_for_initialization, True)
        cost_without = temporaryRouteCost(last_route.customer_list, last_route.vehicle, instance, instance.penalty_cost_iteration_for_initialization, True)
        last_route_cost = cost_with - cost_without


        if len(availableVehicles) > 0:  # only try to pick a new vehicle if we still have vehicles available
            randomVehicle = random.choice(availableVehicles)
            new_route_cost = temporaryRouteCost([0, current_customer, 0], randomVehicle, instance, instance.penalty_cost_iteration_for_initialization, True)
            if last_route_cost <= new_route_cost:
                last_route.customer_list.insert(last_position, current_customer)
            else:
                new_route = RouteObject([0, current_customer, 0], randomVehicle)
                listOfRoutes.append(new_route)
                availableVehicles.remove(randomVehicle)
        else:
            last_route.customer_list.insert(last_position, current_customer)

    return listOfRoutes


def random_sweep(instance: Instance, initialVehicles: list[Vehicle]) -> list[RouteObject]:
    customer_list = sort_customers_by_sweep(instance)

    # emulate changing the starting angle by rotating the list of customers
    rotate = random.randint(0, instance.n - 1)
    customer_list = customer_list[rotate:] + customer_list[:rotate]

    # emulate changing the direction of the sweep by reversing the list of customers
    if random.randint(0, 1) == 1:
        customer_list.reverse()

    return next_fit_heuristic(customer_list, instance, initialVehicles)


def sort_customers_by_sweep(instance: Instance) -> list[int]:
    """
    sorts the customer visits by their angle in a polar coordinate system with the depot at its center.
    To break ties, the distance from the depot is used, then the id of the node itself (which is unique).
    :param instance: corresponding instance
    :return: list of ordered customers sorted by sweeping
    """

    center_x, center_y = instance.coordinates[0]

    customers_angles = []
    for i in range(1, instance.n):
        node_x, node_y = instance.coordinates[i]
        angle = math.atan2(node_y - center_y, node_x - center_x)
        if angle < 0:
            angle += 2.0 * math.pi

        # Update: let be more expressive here and use a dict with named keys
        customers_angles.append({'id': i, 'angle': angle, 'distance': instance.d[0, i]})

    # Update: it's possibility that the order is not clearly defined, so we need break ties in the sorting
    #  (two customers might have the same angle). We introduce the distance to the depot as the second feature
    #  to be sorted against, with the id as a last resort (they are unique).
    #  Tuples are naturally sorted by the first element first, and so on.
    #  We exploit that and provide such a tuple as a key to the sort function

    customers_angles.sort(key=lambda entry: (entry['angle'], entry['distance'], entry['id']))

    sorted_customers = []
    for entry in customers_angles:
        sorted_customers.append(entry['id'])

    return sorted_customers
