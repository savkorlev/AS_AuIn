from instances.Route import RouteObject
from instances.Trucks import Vehicle
from instances.Utils import Instance, temporaryRouteCost, routeCost


def cheapest_insertion_iterative(listOfRoutes: list[RouteObject], listOfRemoved: list[int], list_of_available_vehicles: list[Vehicle], instance: Instance, iteration: int) -> list[Vehicle]:
    # first iteration. create a list_of_customers_to_insert, which will include every removed customer + their regret + their best position + a list of all positions and associated cost
    list_of_customers_to_insert = []

    for cd in listOfRemoved:  # cd -> customers destroyed
        list_of_insert_positions = []

        previousVehicleType = "noVehicle"
        if len(list_of_available_vehicles) > 0:  # only try to create new routes if we still have vehicle available
            for v in list_of_available_vehicles:  # check the cost of a new route
                if v.type != previousVehicleType:  # since we don't have multi-trips, all vehicles with the same type are equal. We therefore only need to check one of each type
                    newCustomerList = [0, cd, 0]
                    newRouteCost = temporaryRouteCost(newCustomerList, v, instance, iteration, True)  # get cost of new route
                    list_of_insert_positions.append((newRouteCost, 'new_route', v))
                    previousVehicleType = v.type

        for i in listOfRoutes:  # iterating over routes in listOfRoutes
            for j in range(len(i.customer_list) - 1):  # iterating over positions in a route
                costWithout = i.current_cost
                temporaryCustomerList = i.customer_list.copy()
                temporaryCustomerList.insert(j + 1, cd)
                costWith = temporaryRouteCost(temporaryCustomerList, i.vehicle, instance, iteration, True)
                insertionCost = costWith - costWithout
                list_of_insert_positions.append((insertionCost, i, j))

        list_of_insert_positions.sort(key=lambda y: y[0], reverse=False)  # sort the list by cost, ascending
        customer_cheapest = list_of_insert_positions[0][0]  # cheapest cost
        customer_best_route = list_of_insert_positions[0][1]  # best route or new_route
        customer_best_position = list_of_insert_positions[0][2]  # best position or best vehicle

        list_of_customers_to_insert.append([cd, customer_cheapest, customer_best_route, customer_best_position, list_of_insert_positions])

    list_of_customers_to_insert.sort(key=lambda y: y[1], reverse=False)  # sort the list by cost, ascending
    best_customer = list_of_customers_to_insert[0][0]
    best_route = list_of_customers_to_insert[0][2]
    best_position = list_of_customers_to_insert[0][3]

    if best_route == "new_route":
        best_vehicle = list_of_customers_to_insert[0][3]
        new_route = RouteObject([0, best_customer, 0], best_vehicle)  # create a new RouteObject
        listOfRoutes.append(new_route)  # add newRoute to list of routes
        list_of_available_vehicles.remove(best_vehicle)
        best_route = new_route  # this way we can check the insertion cost in the new route for all other deleted customers
    else:
        best_route.customer_list.insert(best_position + 1, best_customer)

    best_route.current_cost = routeCost(best_route, instance, iteration, )  # update cost of changed route

    listOfRemoved.remove(best_customer)  # delete current bestCustomer from a list of removed customers
    list_of_customers_to_insert.pop(0)

    # start a loop over all remaining deleted customers. We only recalculate the costs in the routes that have changed (and for new vehicles)
    while len(listOfRemoved) > 0:  # while we still have customers to insert
        for ci in list_of_customers_to_insert:  # ci -> customer insert
            current_customer = ci[0]
            list_of_insert_positions = [pos for pos in ci[4] if pos[1] != best_route and pos[1] != 'new_route']

            for j in range(len(best_route.customer_list) - 1):  # iterating over positions in the changed route
                costWithout = best_route.current_cost
                temporaryCustomerList = best_route.customer_list.copy()
                temporaryCustomerList.insert(j + 1, current_customer)
                costWith = temporaryRouteCost(temporaryCustomerList, best_route.vehicle, instance, iteration, True)
                insertionCost = costWith - costWithout
                list_of_insert_positions.append((insertionCost, best_route, j))

            previousVehicleType = "noVehicle"
            if len(list_of_available_vehicles) > 0:  # only try to create new routes if we still have vehicle available
                for v in list_of_available_vehicles:  # check the cost of a new route
                    if v.type != previousVehicleType:  # since we don't have multi-trips, all vehicles with the same type are equal. We therefore only need to check one of each type
                        newCustomerList = [0, current_customer, 0]
                        newRouteCost = temporaryRouteCost(newCustomerList, v, instance, iteration, True)  # get cost of new route
                        list_of_insert_positions.append((newRouteCost, 'new_route', v))
                        previousVehicleType = v.type

            list_of_insert_positions.sort(key=lambda y: y[0], reverse=False)  # sort the list by cost, ascending
            customer_cheapest = list_of_insert_positions[0][0]  # cheapest cost
            customer_best_route = list_of_insert_positions[0][1]  # best route or new_route
            customer_best_position = list_of_insert_positions[0][2]  # best position or best vehicle

            ci[1] = customer_cheapest
            ci[2] = customer_best_route
            ci[3] = customer_best_position
            ci[4] = list_of_insert_positions

        list_of_customers_to_insert.sort(key=lambda y: y[1], reverse=False)  # sort the list by cost, asscending
        best_customer = list_of_customers_to_insert[0][0]
        best_route = list_of_customers_to_insert[0][2]
        best_position = list_of_customers_to_insert[0][3]

        if best_route == "new_route":
            best_vehicle = list_of_customers_to_insert[0][3]
            new_route = RouteObject([0, best_customer, 0], best_vehicle)  # create a new RouteObject
            listOfRoutes.append(new_route)  # add newRoute to list of routes
            list_of_available_vehicles.remove(best_vehicle)
            best_route = new_route  # this way we can check the insertion cost in the new route for all other deleted customers
        else:
            best_route.customer_list.insert(best_position + 1, best_customer)

        best_route.current_cost = routeCost(best_route, instance, iteration, )  # update cost of changed route

        listOfRemoved.remove(best_customer)  # delete current bestCustomer from a list of removed customers
        list_of_customers_to_insert.pop(0)
        # next iterations

    return list_of_available_vehicles


def regret_insertion(listOfRoutes: list[RouteObject], listOfRemoved: list[int], list_of_available_vehicles: list[Vehicle], instance: Instance, iteration: int) -> list[Vehicle]:
    # first iteration. create a list_of_customers_to_insert, which will include every removed customer + their regret + their best position + a list of all positions and associated cost
    list_of_customers_to_insert = []

    for cd in listOfRemoved:  # cd -> customers destroyed
        list_of_insert_positions = []

        previousVehicleType = "noVehicle"
        if len(list_of_available_vehicles) > 0:  # only try to create new routes if we still have vehicle available
            for v in list_of_available_vehicles:  # check the cost of a new route
                if v.type != previousVehicleType:  # since we don't have multi-trips, all vehicles with the same type are equal. We therefore only need to check one of each type.
                    newCustomerList = [0, cd, 0]
                    newRouteCost = temporaryRouteCost(newCustomerList, v, instance, iteration, True)  # get cost of new route
                    list_of_insert_positions.append((newRouteCost, 'new_route', v))
                    previousVehicleType = v.type

        for i in listOfRoutes:  # iterating over routes in listOfRoutes
            for j in range(len(i.customer_list) - 1):  # iterating over positions in a route
                costWithout = i.current_cost
                temporaryCustomerList = i.customer_list.copy()
                temporaryCustomerList.insert(j + 1, cd)
                costWith = temporaryRouteCost(temporaryCustomerList, i.vehicle, instance, iteration, True)
                insertionCost = costWith - costWithout
                list_of_insert_positions.append((insertionCost, i, j))

        list_of_insert_positions.sort(key=lambda y: y[0], reverse=False)  # sort the list by cost, ascending
        customer_regret = list_of_insert_positions[1][0] - list_of_insert_positions[0][0]  # 2nd cheapest cost - cheapest cost = regret
        customer_best_route = list_of_insert_positions[0][1]  # best route or new_route
        customer_best_position = list_of_insert_positions[0][2]  # best position or best vehicle

        list_of_customers_to_insert.append([cd, customer_regret, customer_best_route, customer_best_position, list_of_insert_positions])

    list_of_customers_to_insert.sort(key=lambda y: y[1], reverse=True)  # sort the list by regret, descending
    best_customer = list_of_customers_to_insert[0][0]
    best_route = list_of_customers_to_insert[0][2]
    best_position = list_of_customers_to_insert[0][3]

    if best_route == "new_route":
        best_vehicle = list_of_customers_to_insert[0][3]
        new_route = RouteObject([0, best_customer, 0], best_vehicle)  # create a new RouteObject
        listOfRoutes.append(new_route)  # add newRoute to list of routes
        list_of_available_vehicles.remove(best_vehicle)
        best_route = new_route  # this way we can check the insertion cost in the new route for all other deleted customers
    else:
        best_route.customer_list.insert(best_position + 1, best_customer)

    best_route.current_cost = routeCost(best_route, instance, iteration, )  # update cost of changed route

    listOfRemoved.remove(best_customer)  # delete current bestCustomer from a list of removed customers
    list_of_customers_to_insert.pop(0)

    # start a loop over all remaining deleted customers. We only recalculate the costs in the routes that have changed (and for new vehicles)
    while len(listOfRemoved) > 0:  # while we still have customers to insert
        for ci in list_of_customers_to_insert:  # ci -> customer insert
            current_customer = ci[0]
            list_of_insert_positions = [pos for pos in ci[4] if pos[1] != best_route and pos[1] != 'new_route']

            for j in range(len(best_route.customer_list) - 1):  # iterating over positions in the changed route
                costWithout = best_route.current_cost
                temporaryCustomerList = best_route.customer_list.copy()
                temporaryCustomerList.insert(j + 1, current_customer)
                costWith = temporaryRouteCost(temporaryCustomerList, best_route.vehicle, instance, iteration, True)
                insertionCost = costWith - costWithout
                list_of_insert_positions.append((insertionCost, best_route, j))

            previousVehicleType = "noVehicle"
            if len(list_of_available_vehicles) > 0:  # only try to create new routes if we still have vehicle available
                for v in list_of_available_vehicles:  # check the cost of a new route
                    if v.type != previousVehicleType:  # since we don't have multi-trips, all vehicles with the same type are equal. We therefore only need to check one of each type.
                        newCustomerList = [0, current_customer, 0]
                        newRouteCost = temporaryRouteCost(newCustomerList, v, instance, iteration, True)  # get cost of new route
                        list_of_insert_positions.append((newRouteCost, 'new_route', v))
                        previousVehicleType = v.type

            list_of_insert_positions.sort(key=lambda y: y[0], reverse=False)  # sort the list by cost, ascending.
            customer_regret = list_of_insert_positions[1][0] - list_of_insert_positions[0][
                0]  # 2nd cheapest cost - cheapest cost = regret
            customer_best_route = list_of_insert_positions[0][1]  # best route or new_route
            customer_best_position = list_of_insert_positions[0][2]  # best position or best vehicle

            ci[1] = customer_regret
            ci[2] = customer_best_route
            ci[3] = customer_best_position
            ci[4] = list_of_insert_positions

        list_of_customers_to_insert.sort(key=lambda y: y[1], reverse=True)  # sort the list by regret, descending
        best_customer = list_of_customers_to_insert[0][0]
        best_route = list_of_customers_to_insert[0][2]
        best_position = list_of_customers_to_insert[0][3]

        if best_route == "new_route":
            best_vehicle = list_of_customers_to_insert[0][3]
            new_route = RouteObject([0, best_customer, 0], best_vehicle)  # create a new RouteObject
            listOfRoutes.append(new_route)  # add newRoute to list of routes
            list_of_available_vehicles.remove(best_vehicle)
            best_route = new_route  # this way we can check the insertion cost in the new route for all other deleted customers
        else:
            best_route.customer_list.insert(best_position + 1, best_customer)

        best_route.current_cost = routeCost(best_route, instance, iteration, )  # update cost of changed route

        listOfRemoved.remove(best_customer)  # delete current bestCustomer from a list of removed customers
        list_of_customers_to_insert.pop(0)
        # next iterations

    return list_of_available_vehicles
