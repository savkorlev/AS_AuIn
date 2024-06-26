import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import random
import math
from docplex.mp.model import Model
from itertools import combinations


results_list = []

def save_results():
    energy_total = 0
    fuel_costs_total = 0
    distance_total = 0
    time_total = 0
    co2_amount = 0

    for r in R:
        if solution.get_value(x[r]) == 1:
            sup = []
            for j in N:
                if solution.get_value(y[j, r]) == 1:
                    sup.append(j)
            for s in F:
                if solution.get_value(u[r, s]) == 1:
                    freq = s
            distance = 0
            for j in V:
                for k in V:
                    for s in F:
                        if k != j:
                            if (solution.get_value(delta[j, k, r, s]) == 1):
                                distance += (c[j, k] * s) / 1000
            time_driving = 0
            time_unloading = 0
            for s in F:
                for j, k in A:
                    for l in L:
                        if j != k:
                            if (solution.get_value(g_e[j, k, r, l, s]) == 1):
                                time_driving += (c[j, k] / v[l]) * s
                            if (solution.get_value(g_c[j, k, r, l, s]) == 1):
                                time_driving += (c[j, k] / v[l]) * s
            for s in F:
                for j in N:
                    if (solution.get_value(sigma_e[j, r, s]) == 1):
                        time_unloading += U[j] * nu[j, s] * s
                    if (solution.get_value(sigma_c[j, r, s]) == 1):
                        time_unloading += U[j] * nu[j, s] * s
            fuel_costs = 0
            weight_energy = 0
            for j in V:
                for k in V:
                    if j != k:
                        for s in F:
                            if (solution.get_value(f_e[j, k, r, s]) > 0):
                                fuel_costs += solution.get_value(f_e[j, k, r, s]) * s * (c_e) * c[
                                    j, k] * alpha * gamma_e
                                weight_energy += solution.get_value(f_e[j, k, r, s]) * s * c[j, k] * alpha * gamma_e
                            if (solution.get_value(delta_e[j, k, r, s]) == 1):
                                fuel_costs += m_e * s * (c_e) * c[j, k] * alpha * gamma_e
                                weight_energy += m_e * s * c[j, k] * alpha * gamma_e
                            if (solution.get_value(f_c[j, k, r, s]) > 0):
                                fuel_costs += solution.get_value(f_c[j, k, r, s]) * s * (c_c + e) * c[
                                    j, k] * alpha * gamma_c
                                weight_energy += solution.get_value(f_c[j, k, r, s]) * s * c[j, k] * alpha * gamma_c
                            if (solution.get_value(delta_c[j, k, r, s]) == 1):
                                fuel_costs += m_c * s * (c_c + e) * c[j, k] * alpha * gamma_c
                                weight_energy += m_c * s * c[j, k] * alpha * gamma_c
            fuel_costs_2 = 0
            speed_energy = 0
            count = 0
            for j in V:
                for k in V:
                    if j != k:
                        for s in F:
                            for l in L:
                                if (solution.get_value(g_e[j, k, r, l, s]) == 1):
                                    fuel_costs_2 += (v[l] / 60) ** 2 * c[j, k] * beta * (c_e) * gamma_e * s
                                    speed_energy += (v[l] / 60) ** 2 * c[j, k] * beta * gamma_e * s
                                if (solution.get_value(g_c[j, k, r, l, s]) == 1):
                                    fuel_costs_2 += (v[l] / 60) ** 2 * c[j, k] * beta * (c_c + e) * gamma_c * s
                                    speed_energy += (v[l] / 60) ** 2 * c[j, k] * beta * gamma_c * s
            truck_costs = 0
            if solution.get_value(t_e[r]) == 1:
                truck_costs += o_e
            if solution.get_value(t_c[r]) == 1:
                truck_costs += o_c
            costs_visit = 0
            for s in F:
                if solution.get_value(u[r, s]) == 1:
                    costs_visit += omega * s

            energy_total += ((weight_energy + speed_energy) * 10 ** (-6) * 0.278)
            fuel_costs_total += (fuel_costs + fuel_costs_2)
            distance_total += distance
            time_total += (time_driving + time_unloading)
            if solution.get_value(t_c[r]) == 1:
                co2_amount += (energy_total * co2)


    obj_val = solution.get_objective_value()
    solv_time = round(mdl.solve_details.time, 3)
    num_routes = 0
    trucks = {'BET': 0, 'ICE': 0}

    for r in R:
        if solution.get_value(x[r]) == 1:
            num_routes += 1
        if solution.get_value(t_e[r]) == 1:
            trucks['BET'] += 1
        if solution.get_value(t_c[r]) == 1:
            trucks['ICE'] += 1

    frequencies = {}
    for r in R:
        for s in F:
            if solution.get_value(u[r, s]) == 1:
                frequencies[r] = s

    speed_levels = {}
    suppliers = {}
    first_supplier = {}
    for r in R:
        speed = []
        if solution.get_value(x[r]) == 1:
            suppliers[r] = []

            for j in V:
                if solution.get_value(y[j, r]) == 1:
                    suppliers[r].append(j)
                for k in V:
                    for s in F:
                        if j != k:
                            if k != 0:
                                if solution.get_value(delta[0, k, r, s]) == 1:
                                    first_supplier[r] = k
                            for l in L:
                                if solution.get_value(g_e[j, k, r, l, s]) == 1:
                                    speed.append(v[l])
                                if solution.get_value(g_c[j, k, r, l, s]) == 1:
                                    speed.append(v[l])
            unique_speed = []
            for level in speed:
                if level not in unique_speed:
                    unique_speed.append(level)
            speed_levels[r] = unique_speed

    obj_val_truck_visits = calculate_truck_visit_cost()

    results_list.append(
        [num_supp, peri, C_e, [obj_val, obj_val_truck_visits], num_routes, suppliers, frequencies, first_supplier, trucks,
         speed_levels, round(energy_total, 2), round(fuel_costs_total, 2), round(distance_total, 2),
         round(time_total, 2), round(co2_amount, 2), solv_time])



def calculate_truck_visit_cost():
    visit_cost = 0
    truck_cost = 0
    for r in R:
        if solution.get_value(t_e[r]) == 1:
            truck_cost += o_e
        if solution.get_value(t_c[r]) == 1:
            truck_cost += o_c
        for s in F:
            if solution.get_value(u[r, s]) == 1:
                visit_cost += omega * s
    return (visit_cost + truck_cost)


peri = 20000    # perimeter, in which the suppliers are arranged around the manufacturer (in meters!)

random.seed(0)
num_supp = 5
supp_list = [[peri/2, peri/2, 0]]   # manufacturer is in the middle
for i in range(1,num_supp+1):
    x = int(random.uniform(0, peri))
    y = int(random.uniform(0, peri))
    t = round(random.uniform(0.3, 0.5), 2)
    supp_list.append([x, y, t])
df_nodes = pd.DataFrame(supp_list, columns = ['x', 'y', 'Unloading Times[min]'])

df_routes = pd.read_csv("data/routes.txt")
df_routes = df_routes.iloc[:(num_supp*4 + 1)]


Q = 20  # vehicle capacity in boxes/paletts
N = []  # set of nodes without the depot
for i in range(1, len(df_nodes)):
    N.append(i)
V = [0] + N  # set of nodes with the depot

S = []   # set of all possible subtours without the depot (needed for constraint 1g)
for L in range(2, len(N)+1):
    for subset in combinations(N, L):
        S.append(subset)

loc_x = df_nodes["x"].to_list()
loc_y = df_nodes["y"].to_list()


A = [(i, j) for i in V for j in V if i != j]  # (0,1), (0,2) and so on - set of arcs
c = {(i, j): np.hypot(loc_x[i]-loc_x[j], loc_y[i]-loc_y[j]) for i, j in A}  # set of costs between arcs - distances


R = []  # set of routes
for i in range(1, len(df_nodes)):
    R.append(i)


P = []  # set of P-LANEs
for i in range(1, df_routes['To[P-LANE]'].nunique() + 1):  # number of unique values = number of P-LANEs
    P.append(i)

b_jp = {}  # number of boxes needed from supplier j ∈ V\{0} on P-LANE p ∈ P (from supplier j ∈ N)
for row, content in df_routes.iterrows():
    b_jp[(content[0], content[1])] = content[2]

b_j = {}  # total number of paletts needed from supplier j in manufacturer’s one-day production
for j in N:
    sum_b_j = 0
    for p in P:
        sum_b_j += b_jp[(j, p)]
    b_j[j] = sum_b_j


d = {p: int(random.uniform(6, 24)) * 60 for p in P}  # the due date of P-LANE p (in minutes)

phi = 5  # transportation cost per unit distance
omega = 40  # fixed cost per vehicle per visit

U = {}  # unloading time per palett for the parts collected from supplier j (in seconds)
for row, content in df_nodes.iloc[1:].iterrows():
    U[row] = content[2]

F = []
F.extend(list(range(1, math.ceil(sum(b_j.values()) / Q) + 1)))

nu = {}  # the average number of boxes collected from supplier j per each visit if supplier j is visited s times
for j in range(1, len(b_j) + 1):
    for s in F:
        nu[(j, s)] = b_j[j]/s

L = [1, 2, 3]  # set of average speed levels (indices)

v = {1: 670, 2: 830, 3:1000}  # set of actual speed levels (40, 50 and 60 km/h in [m/minute])

c_e = 0.06453 * 10 ** (-6)    # electricity costs [€/J]
c_c = 0.0359 * 10 ** (-6) # fuel costs [€/J]
o_e = 325.20    # costs per electric truck [€/day]
o_c = 271.20    # costs per ICE truck [€/day]
alpha = 0.0981  # arc specific energy consumption constant [m/s2]
beta = 3.6123   # vehicle specific energy consumption constant [kg/m]
gamma_c = 12    # correction factor for energy consumption for ICE truck (efficiency, simplifications etc.)
gamma_e = 3.75     # correction factor for energy consumption for electric truck (efficiency, simplifications etc.)
C_e = 1857.6 * 10 ** 6  # electric energy for electric vehicle [J] (=range of 400km)
C_c = 17820 * 10 ** 6    # fuel energy for ICE truck [J] (=range of 1500km)
e = 0.02193 * 10 ** (-7)    # CO2 costs [€/J]
m_e = 4300  # empty weight of electric truck
m_c = 2200  # empty weight of ICE truck
pi = 30.96 * 10 ** 6    # recharging constant for electric truck [J/minute]

co2 = 0.263 # amount of CO2 in diesel [kg/kWh]


l_js = {}  # the average load [kg] collected from supplier j per each visit if supplier j is visited s times
for j in range(1, len(b_j) + 1):
    load_per_palette = int(random.uniform(400, 600))
    for s in F:
        l_js[(j, s)] = b_j[j]/s * load_per_palette


######################################################################################


'''
Summary before the constraints and variables:
Q         vehicle capacity in boxes
N         set of nodes without the depot (set of suppliers)
V         set of nodes with the depot (vertex set)
A         set of arcs (arc set)
c         set of costs (distances) between arcs
R         set of routes
P         set of P-LANEs
b_jp      number of boxes needed from supplier j ∈ V\{0} on P-LANE p ∈ P (from supplier j ∈ N)
b_j       total number of boxes needed from supplier j in manufacturer’s one-day production 
d         the due date of P-LANE p. Time interval from 0 to 12 (check the related todo!)
phi       transportation cost per unit distance
theta     earliness cost
psi       tardiness cost
omega     fixed cost per vehicle per visit
U         unloading time per box for the parts collected from supplier j (in seconds))
F         set of visiting frequencies
nu        the average number of boxes collected from supplier j per each visit if supplier j is visited s times
'''
# Note that we can't use just b because there are b_jp and b_j (same later on for A, F)

mdl = Model('CVRP')

# VARIABLES
x = mdl.binary_var_dict(R, name='x')  # do we select a route
y = mdl.binary_var_dict(((j, r) for j in V for r in R), name='y')  # is the supplier assigned to route? is "for j in N" correct or should it be V? Same applies for sigma, epsilon, F_jp, E and T
z = mdl.binary_var_dict(((j, k, r) for j, k in A for r in R), name='z')  # is arc (j, k) visited by route r?
u = mdl.binary_var_dict(((r, s) for r in R for s in F), name='u')  # is frequency of route r s? Lukas: because of that I think its easier when F is just a list going from 1 to the largest possible visting frequency for any supplier
D = mdl.continuous_var_dict(((r, s) for r in R for s in F), name='D')  # the departure time of the sth visit from the manufacturer for route r Lukas: Frequency again
sigma = mdl.binary_var_dict(((j, r, s) for j in V for r in R for s in F), name='sigma')  # is equal to 1 if y_jr = 1, u_rs = 1 and 0 otherwise
delta = mdl.binary_var_dict(((j, k, r, s) for j, k in A for r in R for s in F), name='delta')  # is equal to 1 if z_jkr = 1, u_rs = 1 and 0 otherwise
M = 10e10  # sufficiently big number Lukas: might be a little bit too big, reducing it would be benefical for solving time, issue of parameter tuning


####################### Extensions ######################################################

f_e = mdl.continuous_var_dict(((j, k, r, s) for j, k in A for r in R for s in F), name='f_e') # weight of boxes transported on arc (j, k)
f_c = mdl.continuous_var_dict(((j, k, r, s) for j, k in A for r in R for s in F), name='f_c') # weight of boxes transported on arc (j, k) on route r with frequency s and an ICE truck
t_e = mdl.binary_var_dict(R, name='t_e')    # = 1 if an electric truck is used in route r
t_c = mdl.binary_var_dict(R, name='t_c')    # = 1 if an ICE truck is used in route r
g_e = mdl.binary_var_dict(((j, k, r, l, s) for j, k in A for r in R for s in F for l in L), name='g_e') # = 1, if average speed on route r with frequency s on arc (j,k) is speed level l with an electric truck
g_c = mdl.binary_var_dict(((j, k, r, l, s) for j, k in A for r in R for s in F for l in L), name='g_c') # = 1, if average speed on route r with frequency s on arc (j,k) is speed level l with an ICE truck
sigma_e = mdl.binary_var_dict(((j, r, s) for j in V for r in R for s in F), name='sigma_e')  # is equal to 1 if y_jr = 1, u_rs = 1, t_e = 1 and 0 otherwise
sigma_c = mdl.binary_var_dict(((j, r, s) for j in V for r in R for s in F), name='sigma_c')  # is equal to 1 if y_jr = 1, u_rs = 1, t_c = 1 and 0 otherwise
E_e = mdl.continuous_var_dict(((j, k) for j, k in A), name = 'E_e')
E_c = mdl.continuous_var_dict(((j, k) for j, k in A), name = 'E_c')
delta_e = mdl.binary_var_dict(((j, k, r, s) for j, k in A for r in R for s in F), name = 'delta_e')
delta_c = mdl.binary_var_dict(((j, k, r, s) for j, k in A for r in R for s in F), name = 'delta_c')

####################################################################################################################

# CONSTRAINTS
# 1b
mdl.add_constraints((x[r + 1] <= x[r] for r in range(1, len(R))), names = '1b')

# 1c
mdl.add_constraints((mdl.sum(y[j, r] for r in R) == 1 for j in N), names = '1c')

# 1d
mdl.add_constraints((mdl.sum(y[j, r] for j in N) >= x[r] for r in R), names = '1d')

# 1e
mdl.add_constraints((mdl.sum(z[j, k, r] for k in V if k != j) == y[j, r] for j in N for r in R), names = '1e')
# if k != j because the distance between the same arcs doesn't exist (KeyError)

# 1f
mdl.add_constraints((mdl.sum(z[j, k, r] for j in V if j != k) == y[k, r] for k in N for r in R), names = '1f')
# if j != k because the distance between the same arcs doesn't exist (KeyError)

# 1g
mdl.add_constraints((mdl.sum(z[j, k, r] for j in subtour for k in subtour if j != k) <= len(subtour)-1 for subtour in S for r in R), names = '1g')

# 1h
mdl.add_constraints((mdl.sum(u[r, s] for s in F) == x[r] for r in R), names = '1h')

# 1i
mdl.add_constraints((mdl.sum(b_j[j] * y[j, r] for j in N) <= mdl.sum(s * Q * u[r, s] for s in F) for r in R), names = '1i')

# 1j
mdl.add_constraints((mdl.sum(b_j[j] * y[j, r] for j in N) >= mdl.sum(((s - 1) * Q + 1) * u[r, s] for s in F) for r in R), names = '1j')

# 1k
mdl.add_constraints((2 * delta[j, k, r, s] <= z[j, k, r] + u[r, s] for j in V for k in V if k != j for r in R for s in F), names = '1k')
# TypeError: cannot unpack non-iterable int object. Reason: for j, k in V -> for j in V for k in V
# if k != j because the distance between the same arcs doesn't exist (KeyError)

# 1l
mdl.add_constraints((1 + delta[j, k, r, s] >= z[j, k, r] + u[r, s] for j in V for k in V if k != j for r in R for s in F), names = '1l')

# 1m
mdl.add_constraints((2 * sigma[j, r, s] <= y[j, r] + u[r, s] for j in N for r in R for s in F), names = '1m')
# Q: is k in sigma indexes a mistake in a paper? A: yes, the same for sigma in 1n

# 1n
mdl.add_constraints((1 + sigma[j, r, s] >= y[j, r] + u[r, s] for j in N for r in R for s in F), names = '1n')


############################### Extensions #################################################################

# load constraint 1
mdl.add_constraints((mdl.sum(f_e[j, k, r, s] for k in V if k != j) - mdl.sum(f_e[k , j, r, s] for k in V if k != j) == l_js[j, s] * sigma_e[j, r, s] for r in R for s in F for j in N), names = 'load-constraint e1')

# load constraint 2
mdl.add_constraints((mdl.sum(f_c[j, k, r , s] for k in V if k != j) - mdl.sum(f_c[k, j, r, s] for k in V if k != j) == l_js[j, s] * sigma_c[j, r, s] for r in R for s in F for j in N), names = 'load-constraint c1')

# load constraint 3
mdl.add_constraints(f_e[j, k, r, s] <= max(l_js.values()) * delta_e[j, k, r, s] for r in R for j, k in A for s in F)

# load constraint 4
mdl.add_constraints(f_c[j, k, r, s] <= max(l_js.values()) * delta_c[j, k, r, s] for r in R for j, k in A for s in F)

# truck type constraint
mdl.add_constraints(t_e[r] + t_c[r] == x[r] for r in R)

# sigma_e constraint 1
mdl.add_constraints(2 * sigma_e[j, r, s] <= sigma[j, r, s] + t_e[r] for r in R for j in V for s in F)

# sigma_c constraint 1
mdl.add_constraints(2 * sigma_c[j, r, s] <= sigma[j, r, s] + t_c[r] for r in R for j in V for s in F)

# sigma_e constraint 2
mdl.add_constraints(1 + sigma_e[j, r, s] >= sigma[j, r, s] + t_e[r] for r in R for j in V for s in F)

# sigma_c constraint 2
mdl.add_constraints(1 + sigma_c[j, r, s] >= sigma[j, r, s] + t_c[r] for r in R for j in V for s in F)

# g_e constraint 1
mdl.add_constraints(2 * mdl.sum(g_e[j, k, r, l, s] for l in L) <= delta[j, k, r, s] + t_e[r] for j, k in A for r in R for s in F)

# g_c constraint 1
mdl.add_constraints(2 * mdl.sum(g_c[j, k, r, l, s] for l in L) <= delta[j, k, r, s] + t_c[r] for j, k in A for r in R for s in F)

# g_e constraint 2
mdl.add_constraints(1 + mdl.sum(g_e[j, k, r, l, s] for l in L) >= delta[j, k, r, s] + t_e[r] for j, k in A for r in R for s in F)

# g_c constraint 2
mdl.add_constraints(1 + mdl.sum(g_c[j, k, r, l, s] for l in L) >= delta[j, k, r, s] + t_c[r] for j, k in A for r in R for s in F)

# ICE range constraint
mdl.add_constraints(mdl.sum(gamma_c * s * (alpha * c[j, k] * (f_c[j, k, r, s] + m_c * delta_c[j, k, r, s]) + beta * c[j, k] * mdl.sum(((v[l] / 60) ** 2) * g_c[j, k, r, l, s] for l in L)) for j, k in A for s in F) <= C_c for r in R)

# electric range constraint
mdl.add_constraints(mdl.sum(gamma_e * s * (alpha * c[j, k] * (f_e[j, k, r, s] + m_e * delta_e[j, k, r, s]) + beta * c[j, k] * mdl.sum(((v[l] / 60) ** 2) * g_e[j, k, r, l, s] for l in L)) for j, k in A for s in F) <= C_e + (240 - mdl.sum((c[j, k] / v[l]) * g_e[j, k, r, l, s] * s for j, k in A for l in L for s in F) - mdl.sum(U[j] * nu[j, s] * s * sigma_e[j, r, s] for j in N for s in F)) * pi for r in R)

# time constraint for electric trucks
mdl.add_constraints(mdl.sum((c[j, k] / v[l]) * g_e[j, k, r, l, s] * s for j, k in A for l in L for s in F) + mdl.sum(U[j] * nu[j, s] * s * sigma_e[j, r, s] for j in N for s in F) <= 240 for r in R)


# time constraint for ICE trucks
mdl.add_constraints(mdl.sum((c[j, k] / v[l]) * g_c[j, k, r, l, s] * s for j, k in A for l in L for s in F) + mdl.sum(U[j] * nu[j, s] * s * sigma_c[j, r, s] for j in N for s in F) <= 240 for r in R)

# delta_e constraint 1
mdl.add_constraints(2 * delta_e[j, k, r, s] <= delta[j, k, r, s] + t_e[r] for j, k in A for r in R for s in F)

# delta_e constraint 2
mdl.add_constraints(1 + delta_e[j, k, r, s] >= delta[j, k, r, s] + t_e[r] for j, k in A for r in R for s in F)

# delta_c constraint 1
mdl.add_constraints(2 * delta_c[j, k, r, s] <= delta[j, k, r, s] + t_c[r] for j, k in A for r in R for s in F)

# delta_c constraint 2
mdl.add_constraints(1 + delta_c[j, k, r, s] >= delta[j, k, r, s] + t_c[r] for j, k in A for r in R for s in F)

# additional flow constraint 1
mdl.add_constraints(mdl.sum(z[0, k, r] for k in N) == x[r] for r in R)


########################################################################################################

# SOLVE AND PRINT (+ EPSILON CONSTRAINT)
mdl.parameters.mip.tolerances.integrality = 0   # forces the binary variables to be excatly 1 or 0 not like 0.999999 or 0.00000003, might lead to a false infeasibility -> increase value slightly then (default = 1e-05); necesarry for Big M formulation of load constraints


################################### Extension objective function ##################################################


# truck costs + visits
obj_tr_vis = mdl.sum(omega * s * u[r, s] for r in R for s in F) + \
           mdl.sum(t_e[r] for r in R) * o_e + \
           mdl.sum(t_c[r] for r in R) * o_c
# # Total costs: 616
# # Total [kWh]: 1057
# # solution.get_objective_value() = 471.2

# pollution/fuel costs
obj_pol_fuel = mdl.sum(((f_e[j, k, r, s] + m_e * delta_e[j, k, r, s]) * c[j, k]) * alpha * c_e * gamma_e * s for j, k in A for r in R for s in F) + \
           mdl.sum(((f_c[j, k, r, s] + m_c * delta_c[j, k, r, s]) * c[j, k]) * alpha * (c_c + e) * gamma_c * s for j, k in A for r in R for s in F) + \
           mdl.sum((mdl.sum((v[l]/60) ** 2 * g_e[j, k, r, l, s] for l in L) * c[j, k] * beta) * c_e * gamma_e * s for j, k in A for r in R for s in F) + \
           mdl.sum((mdl.sum((v[l]/60) ** 2 * g_c[j, k, r, l, s] for l in L) * c[j, k] * beta) * (c_c + e) * gamma_c * s for j, k in A for r in R for s in F)
# # Total costs: 1938
# # Total [kWh]: 130
# # solution.get_objective_value() = 29.6

payoff = {}

mdl.minimize_static_lex([obj_tr_vis, obj_pol_fuel])
solution = mdl.solve(log_output=True)
payoff["transport"] = solution.get_objective_value()

mdl.minimize_static_lex([obj_pol_fuel, obj_tr_vis])
solution = mdl.solve(log_output=True)
payoff["pollution"] = solution.get_objective_value()


ec_l = 14  # intervals
ec_g = ec_l + 1  # grid points
ec_r = payoff["pollution"][1] - payoff["transport"][0]  # range
ec_step = ec_r / ec_l
ec_steps = list(np.arange(payoff["transport"][0], payoff["pollution"][1], ec_step)) + [payoff["pollution"][1]]



obj_values = []
infeasibility_count = 0
ec_delta = 0.000001
ec_s = mdl.continuous_var(name="ec_s")  # we have only one ec_s because we have only one additional objective function to be used as a constraint (obj_pol_fuel)
for e in ec_steps:
    # mdl.minimize_static_lex([obj_pol_fuel - ec_delta * (ec_s / ec_r), obj_tr_vis])
    mdl.minimize(obj_pol_fuel - ec_delta * (ec_s / ec_r))
    mdl.add_constraint(obj_tr_vis + ec_s == e, ctname='ec_constr')
    solution = mdl.solve(log_output=False)
    save_results()
    obj_values.append(solution.get_objective_value())  # append also solution but for now just objective values to compare
    mdl.remove_constraint('ec_constr')

df_results = pd.DataFrame(results_list, columns = ['Number of suppliers', 'Perimeter [m]', 'Battery Capacity [MJ]', 'Object Value (Fuel/Electricity ; Truck+Visits) [€]', 'Number of routes', 'Supplier-route assignment', 'Frequencies', 'First supplier in route', 'Used trucks', 'Speed levels', 'Total energy demand [kWh]', 'Total energy costs [€]', 'Total driven distance [km]', 'Total time [min]', 'Total CO2 amount [kg]', 'Solving time [sec]'])

list_pol =  []
for i in range(len(df_results)):
    list_pol.append(df_results['Object Value (Fuel/Electricity ; Truck+Visits) [€]'][i][0])

list_tr_vis =  []
for i in range(len(df_results)):
    list_tr_vis.append(df_results['Object Value (Fuel/Electricity ; Truck+Visits) [€]'][i][1])


plt.plot(list_pol, list_tr_vis, 'o--', label = 'Pareto front', color = 'b')
plt.ylabel('Objective Value Fixed visit + Truck cost [€]')
plt.xlabel('Objective Value Pollution + Fuel/Electricity cost [€]')
plt.show()
