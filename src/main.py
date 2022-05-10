import argparse
import datetime
import sys
import copy
import time
import pandas as pd
from instances.Construction import ouralgorithm
from instances.Intialization import random_sweep
from instances.Trucks import create_vehicles
from instances.Utils import vehicle_assignment, solution_cost, Instance_tune
from instances.Plot import plotVRP, inner_city_check

# 0. ENTER THE CITY.
# Names to enter: NewYork, Paris, Shanghai
city = "Shanghai"
# set the # of vehicles available to Sweep and Algorithm
numI_Atego = 0
numI_VWTrans = 17
numI_VWCaddy = 0
numI_DeFuso = 0
numI_ScooterL = 0
numI_ScooterS = 0
numI_eCargoBike = 0

# set fixed costs on/off
fixed_cost_active = False  # sets fixed costs active for all vehicles
tax_ins_active = False  # sets taxes and insurance active. Makes fixed costs for all non-leased vehicles ~90% higher (20% for bike)

# set demand_factor
kg_factor = 1
# set volume_factor
vol_factor = 1
# set city cost lvl: 'none', 'low', 'medium', 'high', 'ban'
city_cost_level = 'medium'

# set the run parameters
perform_dict = {}
params_dict = {
    'max_iterations': [5000],
    'init_temp': [0.1],
    'temp_target_percentage': [0.025],
    'temp_target_iteration': [1.2],
    'freeze_period_length': [0.02],
    'destroy_random_ub': [0.12],
    'destroy_expensive_ub': [0.1],
    'destroy_route_ub': [0.5],
    'destroy_related_ub': [0.12],
    'max_weight': [5000],  # Botch: if this is set to 5001, we activate Vehicle_Assignment after Destruction
    'min_weight': [10],
    'reduce_step': [4],
    'step_penalty': [1],
}

# 1. LOADING THE DATA
if city == "NewYork":
    df_nodes = pd.read_csv("data/NewYork.nodes", sep=' ')
    df_nodes["Duration"] = pd.to_timedelta(df_nodes["Duration"]).dt.total_seconds() / 60  # converting duration column to floats instead of strings
    df_routes = pd.read_csv("data/NewYork.routes", sep=' ')
    df_routes["Duration[s]"] = pd.to_timedelta(df_routes["Duration[s]"]).dt.total_seconds() / 60  # converting duration column to floats instead of strings
if city == "Paris":
    df_nodes = pd.read_csv("data/Paris.nodes", sep=' ')
    df_nodes["Duration"] = pd.to_timedelta(df_nodes["Duration"]).dt.total_seconds() / 60  # converting duration column to floats instead of strings
    df_routes = pd.read_csv("data/Paris.routes", sep=' ')
    df_routes["Duration[s]"] = pd.to_timedelta(df_routes["Duration[s]"]).dt.total_seconds() / 60  # converting duration column to floats instead of strings
if city == "Shanghai":
    df_nodes = pd.read_csv("data/Shanghai.nodes", sep=' ')
    df_nodes["Duration"] = pd.to_timedelta(df_nodes["Duration"]).dt.total_seconds() / 60  # converting duration column to floats instead of strings
    df_routes = pd.read_csv("data/Shanghai.routes", sep=' ')
    df_routes["Duration[s]"] = pd.to_timedelta(df_routes["Duration[s]"]).dt.total_seconds() / 60  # converting duration column to floats instead of strings

# 2. CREATING TEST DATASET AND ATTRIBUTES OF FUTURE INSTANCE
testDimension = len(df_nodes)

# DON'T FORGET TO SET MORE VEHICLES IF YOU HAVE MORE CUSTOMERS

if testDimension == len(df_nodes):
    df_nodes_subset = df_nodes  # select all elements (choose if 112 customers)
    df_routes_subset = df_routes  # select all elements (choose if 112 customers)

subsetDemand = list(df_nodes_subset.loc[:, "Demand[kg]"])  # select the demand column (in kg) and convert it to a list

if kg_factor != 1:
    print(f"original demand kg: {subsetDemand}")
    for i in range(len(subsetDemand)):
        subsetDemand[i] = round(subsetDemand[i] * kg_factor)
    print(f"kg factor: {kg_factor}")
    print(f"changed demand kg: {subsetDemand}\n")

subsetVolume = list(df_nodes_subset.loc[:, "Demand[m^3*10^-3]"])  # select the demand column (in volume) and convert it to a list
if vol_factor != 1:
    print(f"original volume: {subsetVolume}")
    for i in range(len(subsetVolume)):
        subsetVolume[i] = round(subsetVolume[i] * kg_factor)
    print(f"volume factor: {vol_factor}")
    print(f"changed volume: {subsetVolume}\n")

subsetDuration = list(df_nodes_subset.loc[:, "Duration"])  # select the duration column (in volume) and convert it to a list

subsetDistances = {}  # create tuples with distances from one Id to another
for row, content in df_routes_subset.iterrows():
    key = (int(content[0][1:]), int(content[1][1:]))
    subsetDistances[key] = content[2]

subsetDistanceInside = {}  # create tuples with distances from one Id to another inside the city
for row, content in df_routes_subset.iterrows():
    key = (int(content[0][1:]), int(content[1][1:]))
    subsetDistanceInside[key] = content[3]

subsetDistanceOutside = {}  # create tuples with distances from one Id to another outside the city
for row, content in df_routes_subset.iterrows():
    key = (int(content[0][1:]), int(content[1][1:]))
    subsetDistanceOutside[key] = content[4]

subsetArcDuration = {}
for row, content in df_routes_subset.iterrows():
    key = (int(content[0][1:]), int(content[1][1:]))
    subsetArcDuration[key] = content[5]

coordinates = []
for row, content in df_nodes_subset.iterrows():
    coordinate = (content[1], content[2])
    coordinates.append(coordinate)

outside_dictionary = inner_city_check(df_nodes_subset, subsetDistanceInside, subsetDistanceOutside)

# 3. CREATING OUR VEHICLES
# create vehicles via function in Trucks.py-file
listOfInitialVehicles = create_vehicles(city, city_cost_level, numI_Atego, numI_VWTrans, numI_VWCaddy, numI_DeFuso, numI_ScooterL, numI_ScooterS, numI_eCargoBike, fixed_cost_active, tax_ins_active)
print(f"List of initial Vehicle payloads_kg: {list(map(lambda x: x.payload_kg, listOfInitialVehicles))}")

# test feasibility of our vehicle assignment. Need enough capacity to carry all demand
sumOfDemand = sum(subsetDemand)
sumOfCapacity = numI_Atego * 2800 + numI_VWTrans * 883 + numI_VWCaddy * 670 + numI_DeFuso * 2800 + numI_ScooterL * 905 + numI_ScooterS * 720 + numI_eCargoBike * 100
if sumOfCapacity < sumOfDemand:
    print(f"Not enough Capacity ({sumOfCapacity}) for Demand ({sumOfDemand})")
    sys.exit()

# 4. RUN THE ALGORITHM
n = 0
for a in params_dict['max_iterations']:
    for b in params_dict['init_temp']:
        for c in params_dict['temp_target_percentage']:
            for d in params_dict['temp_target_iteration']:
                for e in params_dict['freeze_period_length']:
                    for f in params_dict['destroy_random_ub']:
                        for g in params_dict['destroy_expensive_ub']:
                            for h in params_dict['destroy_route_ub']:
                                for i_par in params_dict['destroy_related_ub']:
                                    for j in params_dict['max_weight']:
                                        for k in params_dict['min_weight']:
                                            for l in params_dict['reduce_step']:
                                                for m in params_dict['step_penalty']:
                                                    parser = argparse.ArgumentParser()
                                                    parser.add_argument('--' + str('max_iterations'), default=a)
                                                    parser.add_argument('--' + str('init_temp'), default=b)
                                                    parser.add_argument('--' + str('temp_target_percentage'), default=c)
                                                    parser.add_argument('--' + str('temp_target_iteration'), default=d)
                                                    parser.add_argument('--' + str('freeze_period_length'), default=e)
                                                    parser.add_argument('--' + str('destroy_random_ub'), default=f)
                                                    parser.add_argument('--' + str('destroy_expensive_ub'), default=g)
                                                    parser.add_argument('--' + str('destroy_route_ub'), default=h)
                                                    parser.add_argument('--' + str('destroy_related_ub'), default=i_par)
                                                    parser.add_argument('--' + str('max_weight'), default=j)
                                                    parser.add_argument('--' + str('min_weight'), default=k)
                                                    parser.add_argument('--' + str('reduce_step'), default=l)
                                                    parser.add_argument('--' + str('step_penalty'), default=m)
                                                    args = parser.parse_args()
                                                    ourInstance = Instance_tune(testDimension, listOfInitialVehicles,
                                                                                subsetDemand,
                                                                                subsetVolume,
                                                                                subsetDuration,
                                                                                subsetDistances,
                                                                                subsetDistanceInside,
                                                                                subsetDistanceOutside,
                                                                                subsetArcDuration, coordinates,
                                                                                args)
                                                    bestCostRandomSweep = 10e10
                                                    start_time_run = time.perf_counter()
                                                    for i in range(10):
                                                        tempSolutionRandomSweep = random_sweep(ourInstance,
                                                                                               listOfInitialVehicles)
                                                        tempCost = solution_cost(tempSolutionRandomSweep, ourInstance,
                                                                                 0, True)
                                                        # print(f"Rand Sweep Heuristic, temp distance: {compute_distances(tempSolutionRandomSweep, ourInstance)}")
                                                        if tempCost < bestCostRandomSweep:
                                                            bestSolutionRandomSweep = copy.deepcopy(
                                                                tempSolutionRandomSweep)
                                                            bestCostRandomSweep = tempCost
                                                    listOfInitAvailableVehicles = vehicle_assignment(
                                                        bestSolutionRandomSweep,
                                                        listOfInitialVehicles, ourInstance,
                                                        0, True)
                                                    sol, final_cost, feasible = ouralgorithm(ourInstance, bestSolutionRandomSweep,
                                                                                   listOfInitialVehicles,
                                                                                   listOfInitAvailableVehicles,
                                                                                   coordinates)
                                                    end_time_run = time.perf_counter()
                                                    runtime_run = end_time_run - start_time_run
                                                    print(f"numI_Atego: {numI_Atego,}, numI_VWTrans: {numI_VWTrans}, numI_VWCaddy: {numI_VWCaddy}, numI_DeFuso: {numI_DeFuso}, numI_ScooterL: {numI_ScooterL}, numI_ScooterS: {numI_ScooterS}, numI_eCargoBike: {numI_eCargoBike}")
                                                    no_depot_title = 'No Depot Plot ' + str(n)
                                                    depot_title = 'Route Plot in Permutation ' + str(n)
                                                    # plotVRP(sol, coordinates, False, no_depot_title)
                                                    plotVRP(sol, coordinates, outside_dictionary, True, depot_title)

                                                    perform_dict[n] = [a, b, c, d, e, f, g, h, i_par, j, k, l, m, feasible, fixed_cost_active, tax_ins_active, final_cost, runtime_run]
                                                    n += 1

summary_performance = pd.DataFrame.from_dict(perform_dict, orient='index', columns=list(params_dict.keys()) + ['feasible'] + ['fixed costs'] +['tax+ins'] + ['cost'] + ['runtime in s'])
print()
print(summary_performance)

date_string = str(datetime.datetime.now())
date_string = date_string.replace(":", "-")
summary_performance.to_csv(date_string[:16] + ' - parameter_analysis.csv')
