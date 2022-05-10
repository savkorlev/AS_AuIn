import datetime
import random
import copy
import time
from typing import List
from instances.DestructionOps import random_removal, expensive_removal, route_removal, related_removal
from instances.InsertionOps import cheapest_insertion_iterative, regret_insertion
from instances.LocalSearch import hillclimbing, find_best_improvement_2Opt, combine_routes
from instances.Plot import plot3Subplots
from instances.Route import RouteObject
from instances.Trucks import Vehicle
from instances.Utils import Instance, routeCost, delete_empty_routes, vehicle_assignment, solution_cost, \
    simulated_annealing, blockPrint, \
    enablePrint, compute_distances_objects, compute_total_demand, compute_duration, compute_total_volume, \
    compute_distance


def ouralgorithm(instance: Instance, initialSolution: List[RouteObject], list_of_all_vehicles: List[Vehicle], listOfInitAvailableVehicles: List[Vehicle], coordinates_int=[]):  # coordinates_int is only for matplot

    # START OF INITIALIZATION PHASE
    starttime = datetime.datetime.now()
    feasible_solution_found = False  # to track if we found a feasible solution
    accepted_list_of_av_vehicles = listOfInitAvailableVehicles.copy()
    # setting the initial solution up so we can compare to it in acceptance phase
    bestSolution = copy.deepcopy(initialSolution)  # set the initial solution as the best solution (until acceptance check)
    bestIteration = -1  # used in acceptance check

    # setting up weights, counters and lists for adaptive LNS and generating reports after algorithm finishes
    weight_destroy_random = instance.init_weight_destroy_random  # set initial weight for operation
    counter_destroy_random_imp = 0
    counter_destroy_random_rej = 0
    weight_destroy_expensive = instance.init_weight_destroy_expensive
    counter_destroy_expensive_imp = 0
    counter_destroy_expensive_rej = 0
    weight_destroy_route = instance.init_weight_destroy_route
    counter_destroy_route_imp = 0
    counter_destroy_route_rej = 0
    weight_destroy_related = instance.init_weight_destroy_related
    counter_destroy_related_imp = 0
    counter_destroy_related_rej = 0
    weight_insert_cheapest = instance.init_weight_insert_cheapest
    counter_insert_cheapest_imp = 0
    counter_insert_cheapest_rej = 0
    weight_insert_regret = instance.init_weight_insert_regret
    counter_insert_regret_imp = 0
    counter_insert_regret_rej = 0

    listImprovingIterations = []  # collects all iterations where we found an improvement

    # setting up counter to check how long we had no improvement
    counter_iterations_no_improvement = 0
    freeze_iterations = 0

    # set up parameters of simulated annealing
    currentSolution = copy.deepcopy(initialSolution)
    currentCost = solution_cost(initialSolution, instance, iteration=0, penalty_active=True)
    temperature = instance.init_temp * currentCost  # current cost is used to calculate initial temperature
    accept_counter = 0
    simAnnPlot = []  # to store our results for a plot
    simAnnTemp = []
    bestSolutionPlot = []

    print(f"Sweep solution: {list(map(lambda x: x.customer_list, bestSolution))}")  # printing out customer lists after sweep
    print(f"Route costs:    {list(map(lambda x: x.current_cost, bestSolution))}")  # printing out costs of the routes after sweep after costs are assigned
    # END OF INITIALIZATION PHASE

    # ------------------------------------------------------------------------------------------------------------------

    # START OF THE LOOP
    iteration = 0  # iterations are counted up at the start of the loop, so we start with 0 to have the first iteration = 1
    perf_starttime = time.perf_counter()  # starts the timer for our maxTime. Does not include the initialization. This is ok since the sweep is very short.
    time_so_far = 0.0

    while time_so_far < instance.max_time and iteration < instance.max_iterations:  # run until either maxIterations or maxTime is reached. Will do 1 last loop after maxTime.
        iteration += 1  # count up the iterations

        blockPrint()
        if iteration % 50 == 0 or iteration == 1:
            enablePrint()

        print(f"New iteration__________{iteration}")

        listOfRoutes = copy.deepcopy(currentSolution)  # at the start of each iteration, set the list of routes to current known solution
        list_of_available_vehicles = accepted_list_of_av_vehicles.copy()  # at the start of each iteration, set the list_of_avb_vehicles to the list from the accepted solution

        print(f"Routes at start of iteration:     {list(map(lambda x: x.customer_list, listOfRoutes))}")

        # --------------------------------------------------------------------------------------------------------------

        # START OF DESTRUCTION PHASE
        destroy_ops = ['random_removal', 'expensive_removal', 'route_removal', 'related_removal']
        destroy_weights = [weight_destroy_random, weight_destroy_expensive, weight_destroy_route, weight_destroy_related]
        destroy_op_used_list = random.choices(destroy_ops, weights=destroy_weights)  # chooses an option from a weighed list
        destroy_op_used = destroy_op_used_list[0]  # because the choices-operator returns a list

        if destroy_op_used == 'random_removal':  # pick a destroy operation
            listOfRemoved = random_removal(instance)
        elif destroy_op_used == 'expensive_removal':
            listOfRemoved = expensive_removal(listOfRoutes, instance, iteration)
        elif destroy_op_used == 'route_removal':
            listOfRemoved = route_removal(listOfRoutes, instance)
        elif destroy_op_used == 'related_removal':
            listOfRemoved = related_removal(instance)

        listOfRemoved.sort()  # for better readability
        print(f"Customers to be removed:          {listOfRemoved}")

        temp_listOfRemoved = listOfRemoved.copy()
        for r in listOfRoutes:
            for rc in temp_listOfRemoved:
                if rc in r.customer_list:
                    r.customer_list.remove(rc)
        print(f"Routes after destruction:         {list(map(lambda x: x.customer_list, listOfRoutes))}")

        for r in listOfRoutes:
            r.current_cost = routeCost(r, instance, iteration, True)  # recalculate route cost after destruction
        # END OF DESTRUCTION PHASE

        #  -------------------------------------------------------------------------------------------------------------

        # START OF INSERTION PHASE
        insert_ops = ['cheapest_insert', 'regret_insert']
        insert_weights = [weight_insert_cheapest, weight_insert_regret]
        insert_op_used_list = random.choices(insert_ops, weights=insert_weights)  # chooses an option from a weighed list
        insert_op_used = insert_op_used_list[0]  # because the choices-operator returns a list

        if insert_op_used == 'cheapest_insert':  # pick a destroy operation
            list_of_available_vehicles = cheapest_insertion_iterative(listOfRoutes, listOfRemoved, list_of_available_vehicles, instance, iteration)
        elif insert_op_used == 'regret_insert':
            list_of_available_vehicles = regret_insertion(listOfRoutes, listOfRemoved, list_of_available_vehicles, instance, iteration)

        print(f"Route objects after insertion:    {list(map(lambda x: x.customer_list, listOfRoutes))}")
        # END OF INSERTION PHASE

        # --------------------------------------------------------------------------------------------------------------

        # START OF OPTIMIZATION PHASE
        listOfRoutes = delete_empty_routes(listOfRoutes)
        print(f"Route objects no empty routes:    {list(map(lambda x: x.customer_list, listOfRoutes))}")
        local_search_function = find_best_improvement_2Opt
        listAfterOptimization = hillclimbing(list(map(lambda x: x.customer_list, listOfRoutes)), instance, local_search_function)
        for i in range(len(listAfterOptimization)):
            listOfRoutes[i].customer_list = listAfterOptimization[i].copy()  # put the optimized customer lists back into our RouteObjects
        print(f"Route objects after optimization: {list(map(lambda x: x.customer_list, listOfRoutes))}")

        listOfRoutes, list_of_available_vehicles = combine_routes(listOfRoutes, list_of_available_vehicles, instance, iteration)

        # START OF VEHICLE SWAP PHASE
        list_of_available_vehicles = vehicle_assignment(listOfRoutes, list_of_all_vehicles, instance, iteration)
        # END OF VEHICLE SWAP PHASE

        # END OF OPTIMIZATION PHASE.

        # --------------------------------------------------------------------------------------------------------------

        # START OF ACCEPTANCE PHASE
        # count customers
        customer_count = 0
        for r in listOfRoutes:
            customer_count += len(r.customer_list) - 2  # -2 because of two depots in each customer_list
        print(f"customer count check: {customer_count}")

        bestCost = solution_cost(bestSolution, instance, iteration, True)  # we have to recalculate best cost after every iteration because we need to update infeasibility costs
        print(f"Best known cost before this iteration: {bestCost}")
        print(f"Best iteration before this one: {bestIteration}")
        # costThisIteration = solution_cost(listOfRoutes, instance, iteration, True)
        accept, costThisIteration, temperature, freeze_iterations = simulated_annealing(instance, currentSolution, listOfRoutes, temperature, iteration, freeze_iterations)

        accept_counter += accept
        if accept:
            currentSolution = listOfRoutes
            accepted_list_of_av_vehicles = list_of_available_vehicles.copy()
            simAnnPlot.append((iteration, costThisIteration))
            freeze_iterations = max(1, round(
                instance.freeze_period_length * iteration))  # we freeze the simulated annealing after we accept a solution to allow local optimization of our new solution

        if freeze_iterations > 0:
            simAnnTemp.append((iteration, 0))
        else:
            simAnnTemp.append((iteration, temperature))

        if iteration == (1 - instance.final_effort) * instance.max_iterations:  # this statement forces our algorithm back to the best known solution at 99% of iterations. This helps us optimize the bestSolution for a little bit longer in the end.
            currentSolution = copy.deepcopy(bestSolution)
            freeze_iterations = instance.max_iterations  # we freeze the SimAnnealing at the end so we only accept better solutions

        if costThisIteration < bestCost:
            bestCost = costThisIteration
            bestSolution = copy.deepcopy(listOfRoutes)
            bestIteration = iteration
            formated_cost = "{:.2f}".format(bestCost)

            solution_feasible = True
            for route in bestSolution:  # checking if all our routes are feasible
                if not route.currently_feasible:
                    solution_feasible = False
            bestSolutionPlot.append((iteration, costThisIteration, solution_feasible))
            if solution_feasible:
                best_feasible_solution = copy.deepcopy(bestSolution)  # stores the best feasible solution
                feasible_solution_found = True

            listImprovingIterations.append((iteration, formated_cost, solution_feasible))

            counter_iterations_no_improvement = 0  # reset the counter

            if destroy_op_used == 'random_removal':
                weight_destroy_random = min(instance.max_weight, weight_destroy_random + iteration)
                counter_destroy_random_imp += 1
            elif destroy_op_used == 'expensive_removal':
                weight_destroy_expensive = min(instance.max_weight, weight_destroy_expensive + iteration)
                counter_destroy_expensive_imp += 1
            elif destroy_op_used == 'route_removal':
                weight_destroy_route = min(instance.max_weight, weight_destroy_random + iteration)
                counter_destroy_route_imp += 1
            elif destroy_op_used == 'related_removal':
                weight_destroy_related = min(instance.max_weight, weight_destroy_related + iteration)
                counter_destroy_related_imp += 1

            if insert_op_used == 'cheapest_insert':
                weight_insert_cheapest = min(instance.max_weight, weight_insert_cheapest + iteration)
                counter_insert_cheapest_imp += 1
            elif insert_op_used == 'regret_insert':
                weight_insert_regret = min(instance.max_weight, weight_insert_regret + iteration)
                counter_insert_regret_imp += 1

        else:  # if we dont find a better solution solution:
            counter_iterations_no_improvement += 1

            if destroy_op_used == 'random_removal':  # pick a destroy operation
                weight_destroy_random = max(instance.min_weight, weight_destroy_random - instance.reduce_step)
                counter_destroy_random_rej += 1
            elif destroy_op_used == 'expensive_removal':
                weight_destroy_expensive = max(instance.min_weight, weight_destroy_expensive - instance.reduce_step)
                counter_destroy_expensive_rej += 1
            elif destroy_op_used == 'route_removal':  # pick a destroy operation
                weight_destroy_route = max(instance.min_weight, weight_destroy_route - instance.reduce_step)
                counter_destroy_route_rej += 1
            elif destroy_op_used == 'related_removal':
                weight_destroy_related = max(instance.min_weight, weight_destroy_related - instance.reduce_step)
                counter_destroy_related_rej += 1

            if insert_op_used == 'cheapest_insert':  # pick a destroy operation
                weight_insert_cheapest = max(10, weight_insert_cheapest - instance.reduce_step)
                counter_insert_cheapest_rej += 1
            elif insert_op_used == 'regret_insert':
                weight_insert_regret = max(10, weight_insert_regret - instance.reduce_step)
                counter_insert_regret_rej += 1

        print(f"Total cost of the current iteration: {costThisIteration}, Feasible: {solution_feasible}")
        print(f"Best known cost: {bestCost}")
        print(f"Best iteration: {bestIteration}, iterations without improvement: {counter_iterations_no_improvement}")
        print(f"End of iteration__________{iteration}\n")

        time_so_far = time.perf_counter() - perf_starttime  # update time for maxTime
        # END OF ACCEPTANCE PHASE

    # -------------------------------------------------------------------------------------------------------------

    # END OF LOOP
    enablePrint()
    print(f"Routes of the best Solution:")
    counter = 0
    for r in bestSolution:
        counter += 1
        print(f"Best Solution - route {counter} , vehicle {r.vehicle.plateNr}, cost: {r.current_cost:.2f} €, load: {compute_total_demand(r.customer_list, instance)} kg, vol: {compute_total_volume(r.customer_list, instance) / 1000:.2f} m^3, dist: {compute_distance(r.customer_list, instance):.0f} km, customerCount: {len(r.customer_list) - 2}, feasible: {r.currently_feasible}, customers: {r.customer_list}")
    print()

    print(f"random_removal stats: improvements: {counter_destroy_random_imp}, rejected: {counter_destroy_random_rej}")
    print(f"expensive_removal stats: improvements: {counter_destroy_expensive_imp}, rejected: {counter_destroy_expensive_rej}")
    print(f"route_removal stats: improvements: {counter_destroy_route_imp}, rejected: {counter_destroy_route_rej}")
    print(f"related_removal stats: improvements: {counter_destroy_related_imp}, rejected: {counter_destroy_related_rej}")
    print(f"Weights at end of run: random_removal: {weight_destroy_random}, expensive_removal: {weight_destroy_expensive}, route_removal: {weight_destroy_route}, related_removal: {weight_destroy_related}")

    print(f"cheapest_insert stats: improvements: {counter_insert_cheapest_imp}, rejected: {counter_insert_cheapest_rej}")
    print(f"regret_insert stats: improvements: {counter_insert_regret_imp}, rejected: {counter_insert_regret_rej}")
    print(f"Weights at end of run: cheapest_insert: {weight_insert_cheapest}, regret_insert: {weight_insert_regret}")
    print()

    initialCost = solution_cost(initialSolution, instance, iteration)
    improvement = 100 - ((bestCost / initialCost) * 100)
    imp_per_it = improvement / (iteration + 1)
    print(f"Finished after iteration {iteration}")
    print(f"Initialization cost: {initialCost:.2f}, ourAlgorithm cost: {bestCost:.2f}.")

    final_distance = compute_distances_objects(bestSolution, instance)
    final_duration = 0.0
    for r in bestSolution:
        final_duration += compute_duration(r.customer_list, instance)
    print(f"Final cumulative distance for comparison: {final_distance}")
    print(f"Final cumulative duration for comparison: {final_duration}")

    print(f"We improved by {improvement:.2f}%. Average improvement per iteration: {imp_per_it:.2f}%.")
    print(f"We improved {len(listImprovingIterations)} times, in the following iterations: {listImprovingIterations}.")
    print('accept: ' + str(accept_counter) + ', iterations: ' + str(iteration) + ', ratio: ' + str(accept_counter / iteration))
    endtime = datetime.datetime.now()
    print(f"Length of the run: {endtime - starttime}.\n")
    print(str(endtime))

    formated_cost = "{:.2f}".format(bestCost)
    plot3Subplots(simAnnPlot, bestSolutionPlot, simAnnTemp, 'SimAnn Accepted / Temp - Best Cost: ' + formated_cost)

    if feasible_solution_found:
        return best_feasible_solution, bestCost, feasible_solution_found
    else:
        print(f"FOUND NO FEASIBLE SOLUTION!")
        return bestSolution, bestCost, feasible_solution_found
